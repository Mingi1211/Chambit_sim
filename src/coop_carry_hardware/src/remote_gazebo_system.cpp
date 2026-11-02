#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_set>
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace coop_carry_hardware
{
using Clock = std::chrono::steady_clock;
using Message = sensor_msgs::msg::JointState;
struct Sample {std::vector<double> q, dq; Clock::time_point received; int64_t stamp;};
struct Command {std::vector<double> q; Clock::time_point written;};

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
      names_.push_back(joint.name);
      const bool driven = !joint.command_interfaces.empty();
      driven_.push_back(driven);
    }
    auto node = get_node();
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    state_subscription_ = node->create_subscription<Message>(
      info.hardware_parameters.at("state_topic"), rclcpp::SensorDataQoS(),
      [this](Message::ConstSharedPtr msg) {
        // Reject malformed, incomplete, repeated, or reversed source measurements.
        const int64_t stamp = int64_t(msg->header.stamp.sec) * 1000000000LL +
          msg->header.stamp.nanosec;
        if (stamp <= last_stamp_ || msg->name.size() != msg->position.size() ||
          msg->name.size() != msg->velocity.size() ||
          std::unordered_set<std::string>(msg->name.begin(), msg->name.end()).size() != msg->name.size()) {return;}
        const double source_age = get_node()->now().seconds() - stamp / 1e9;
        if (source_age > timeout_ || source_age < -0.1) {return;}
        auto value = std::make_shared<Sample>();
        value->stamp = stamp;
        value->received = Clock::now();
        for (const auto & name : names_) {
          auto it = std::find(msg->name.begin(), msg->name.end(), name);
          if (it == msg->name.end()) {return;}
          const auto index = std::distance(msg->name.begin(), it);
          if (!std::isfinite(msg->position[index]) || !std::isfinite(msg->velocity[index])) {
            return;
          }
          value->q.push_back(msg->position[index]);
          value->dq.push_back(msg->velocity[index]);
        }
        last_stamp_ = stamp;
        std::lock_guard<std::mutex> lock(state_mutex_);
        sample_ = value;
      });
    publisher_ = node->create_publisher<Message>(
      info.hardware_parameters.at("command_topic"), rclcpp::QoS(1));
    // Publishing runs on the executor, never in the controller's read/write loop.
    timer_ = node->create_wall_timer(std::chrono::milliseconds(10), [this]() {
      std::shared_ptr<const Command> command;
      std::shared_ptr<const Sample> state;
      {std::lock_guard<std::mutex> lock(command_mutex_); command = command_;}
      {std::lock_guard<std::mutex> lock(state_mutex_); state = sample_;}
      if (!active_ || !command || !state ||
        std::chrono::duration<double>(Clock::now() - command->written).count() > timeout_ ||
        std::chrono::duration<double>(Clock::now() - state->received).count() > timeout_) {return;}
      Message msg;
      msg.header.stamp = get_node()->now();
      for (size_t i = 0; i < names_.size(); ++i) {
        if (driven_[i]) {msg.name.push_back(names_[i]); msg.position.push_back(command->q[i]);}
      }
      publisher_->publish(msg);
    });
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    std::shared_ptr<const Sample> value;
    {std::lock_guard<std::mutex> lock(state_mutex_); value = sample_;}
    if (!value || std::chrono::duration<double>(Clock::now() - value->received).count() > timeout_) {
      RCLCPP_ERROR(get_logger(), "Remote hardware activation needs fresh, complete Gazebo feedback");
      return hardware_interface::CallbackReturn::FAILURE;
    }
    for (size_t i = 0; i < names_.size(); ++i) {
      set_state(names_[i] + "/position", value->q[i]);
      set_state(names_[i] + "/velocity", value->dq[i]);
      if (driven_[i]) {set_command(names_[i] + "/position", value->q[i]);}
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
    {std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {read_sample_ = sample_;}}
    const auto & value = read_sample_;
    if (!value) {return hardware_interface::return_type::OK;}
    if (active_ && std::chrono::duration<double>(Clock::now() - value->received).count() > timeout_) {
      RCLCPP_ERROR(get_logger(), "Remote source feedback stopped; deactivate hardware/controllers");
      active_ = false;
      return hardware_interface::return_type::ERROR;
    }
    for (size_t i = 0; i < names_.size(); ++i) {
      set_state(names_[i] + "/position", value->q[i]);
      set_state(names_[i] + "/velocity", value->dq[i]);
    }
    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) {return hardware_interface::return_type::OK;}
    auto value = std::make_shared<Command>();
    value->written = Clock::now();
    for (size_t i = 0; i < names_.size(); ++i) {
      const double q = driven_[i] ? get_command(names_[i] + "/position") : 0.0;
      if (!std::isfinite(q)) {return hardware_interface::return_type::ERROR;}
      value->q.push_back(q);
    }
    {std::unique_lock<std::mutex> lock(command_mutex_, std::try_to_lock);
      if (lock.owns_lock()) {command_ = value;}}
    return hardware_interface::return_type::OK;
  }
private:
  std::vector<std::string> names_;
  std::vector<bool> driven_;
  double timeout_ = 0.5;
  std::atomic<bool> active_{false};
  int64_t last_stamp_ = -1;
  std::mutex state_mutex_, command_mutex_;
  std::shared_ptr<const Sample> sample_, read_sample_;
  std::shared_ptr<const Command> command_;
  rclcpp::Subscription<Message>::SharedPtr state_subscription_;
  rclcpp::Publisher<Message>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace coop_carry_hardware
PLUGINLIB_EXPORT_CLASS(coop_carry_hardware::RemoteGazeboSystem, hardware_interface::SystemInterface)
