#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace coop_carry_hardware
{
using Clock = std::chrono::steady_clock;
using Message = sensor_msgs::msg::JointState;
struct Sample
{
  std::vector<double> positions;
  std::vector<double> velocities;
  Clock::time_point received_at;
  int64_t stamp;
};
struct Command
{
  std::vector<double> positions;
  Clock::time_point written_at;
};

// Only the simulation transport changes. Existing JTC interfaces remain position/velocity.
class RemoteGazeboSystem : public hardware_interface::SystemInterface
{
 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto & info = get_hardware_info();
    timeout_ = std::stod(info.hardware_parameters.at("state_timeout"));
    for (const auto & joint : info.joints) {
      joint_names_.push_back(joint.name);
      const bool driven = !joint.command_interfaces.empty();
      commanded_joints_.push_back(driven);
    }
    auto node = get_node();
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    state_subscription_ = node->create_subscription<Message>(
        info.hardware_parameters.at("state_topic"), rclcpp::SensorDataQoS(),
        [this](Message::ConstSharedPtr msg) { receive_state(msg); });
    publisher_ = node->create_publisher<Message>(info.hardware_parameters.at("command_topic"),
                                                 rclcpp::QoS(1));
    // Publishing runs on the executor, never in the controller's read/write loop.
    timer_ =
        node->create_wall_timer(std::chrono::milliseconds(10), [this]() { publish_command(); });
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    std::shared_ptr<const Sample> value;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      value = sample_;
    }
    if (!value ||
        std::chrono::duration<double>(Clock::now() - value->received_at).count() > timeout_) {
      RCLCPP_ERROR(get_logger(),
                   "Remote hardware activation needs fresh, complete Gazebo feedback");
      return hardware_interface::CallbackReturn::FAILURE;
    }
    // 시작 시 실제 관절 위치를 명령 초기값으로 사용해 목표가 튀지 않게 한다.
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      set_state(joint_names_[i] + "/position", value->positions[i]);
      set_state(joint_names_[i] + "/velocity", value->velocities[i]);
      if (commanded_joints_[i]) {
        set_command(joint_names_[i] + "/position", value->positions[i]);
      }
    }
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // No waiting on callbacks in the control loop: reuse the last immutable snapshot
    // if the very short callback exchange is in progress.
    {
      std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        read_sample_ = sample_;
      }
    }
    const auto & value = read_sample_;
    if (!value) {
      return hardware_interface::return_type::OK;
    }
    if (active_ &&
        std::chrono::duration<double>(Clock::now() - value->received_at).count() > timeout_) {
      RCLCPP_ERROR(get_logger(), "Remote source feedback stopped; deactivate hardware/controllers");
      active_ = false;
      return hardware_interface::return_type::ERROR;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      set_state(joint_names_[i] + "/position", value->positions[i]);
      set_state(joint_names_[i] + "/velocity", value->velocities[i]);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) {
      return hardware_interface::return_type::OK;
    }
    auto value = std::make_shared<Command>();
    value->written_at = Clock::now();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const double q = commanded_joints_[i] ? get_command(joint_names_[i] + "/position") : 0.0;
      if (!std::isfinite(q)) {
        return hardware_interface::return_type::ERROR;
      }
      value->positions.push_back(q);
    }
    {
      std::unique_lock<std::mutex> lock(command_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        command_ = value;
      }
    }
    return hardware_interface::return_type::OK;
  }

 private:
  // 수신 콜백에서 검증·정렬을 끝내고, 제어 루프에는 완성된 상태만 넘긴다.
  void receive_state(Message::ConstSharedPtr msg)
  {
    // Reject malformed, incomplete, repeated, or reversed source measurements.
    const int64_t stamp = int64_t(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
    if (stamp <= last_source_stamp_ || msg->name.size() != msg->position.size() ||
        msg->name.size() != msg->velocity.size() ||
        std::unordered_set<std::string>(msg->name.begin(), msg->name.end()).size() !=
            msg->name.size()) {
      return;
    }
    const double source_age = get_node()->now().seconds() - stamp / 1e9;
    if (source_age > timeout_ || source_age < -0.1) {
      return;
    }
    auto value = std::make_shared<Sample>();
    value->stamp = stamp;
    value->received_at = Clock::now();
    for (const auto & name : joint_names_) {
      auto it = std::find(msg->name.begin(), msg->name.end(), name);
      if (it == msg->name.end()) {
        return;
      }
      const auto index = std::distance(msg->name.begin(), it);
      if (!std::isfinite(msg->position[index]) || !std::isfinite(msg->velocity[index])) {
        return;
      }
      value->positions.push_back(msg->position[index]);
      value->velocities.push_back(msg->velocity[index]);
    }
    last_source_stamp_ = stamp;
    std::lock_guard<std::mutex> lock(state_mutex_);
    sample_ = value;
  }

  // ROS 발행은 제어 루프가 아니라 10ms 타이머에서 수행한다.
  void publish_command()
  {
    std::shared_ptr<const Command> command;
    std::shared_ptr<const Sample> state;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = command_;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state = sample_;
    }
    if (!active_ || !command || !state ||
        std::chrono::duration<double>(Clock::now() - command->written_at).count() > timeout_ ||
        std::chrono::duration<double>(Clock::now() - state->received_at).count() > timeout_) {
      return;
    }
    Message msg;
    msg.header.stamp = get_node()->now();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (commanded_joints_[i]) {
        msg.name.push_back(joint_names_[i]);
        msg.position.push_back(command->positions[i]);
      }
    }
    publisher_->publish(msg);
  }

  std::vector<std::string> joint_names_;
  std::vector<bool> commanded_joints_;
  double timeout_ = 0.5;
  std::atomic<bool> active_{false};
  int64_t last_source_stamp_ = -1;
  std::mutex state_mutex_;
  std::mutex command_mutex_;
  std::shared_ptr<const Sample> sample_;
  std::shared_ptr<const Sample> read_sample_;
  std::shared_ptr<const Command> command_;
  rclcpp::Subscription<Message>::SharedPtr state_subscription_;
  rclcpp::Publisher<Message>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace coop_carry_hardware
PLUGINLIB_EXPORT_CLASS(coop_carry_hardware::RemoteGazeboSystem, hardware_interface::SystemInterface)
