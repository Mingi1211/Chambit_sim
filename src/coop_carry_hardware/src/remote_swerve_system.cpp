// 외부 Controller Manager의 조향 위치·구동 속도를 Gazebo 전용 통신으로 연결한다.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "actuator_msgs/msg/actuators.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace coop_carry_hardware
{
using SwerveClock = std::chrono::steady_clock;

class RemoteSwerveSystem : public hardware_interface::SystemInterface
{
  struct Sample
  {
    std::array<double, 6> position{}, velocity{};
    SwerveClock::time_point received;
  };

 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto & info = get_hardware_info();
    if (info.joints.size() != 6) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (size_t i = 0; i < 6; ++i) {
      names_[i] = info.joints[i].name;
      const auto expected = i < 3 ? "position" : "velocity";
      if (info.joints[i].command_interfaces.size() != 1 ||
          info.joints[i].command_interfaces[0].name != expected) {
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    timeout_ = std::stod(info.hardware_parameters.at("state_timeout"));
    auto node = get_node();
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    publisher_ = node->create_publisher<actuator_msgs::msg::Actuators>(
        info.hardware_parameters.at("command_topic"), 1);
    subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
        info.hardware_parameters.at("state_topic"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { receive(msg); });
    timer_ = node->create_wall_timer(std::chrono::milliseconds(10), [this]() { publish(); });
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!sample_ || age(sample_->received) > timeout_) {
      return hardware_interface::CallbackReturn::FAILURE;
    }
    for (size_t i = 0; i < 6; ++i) {
      set_state(names_[i] + "/position", sample_->position[i]);
      set_state(names_[i] + "/velocity", sample_->velocity[i]);
      set_command(names_[i] + (i < 3 ? "/position" : "/velocity"),
                  i < 3 ? sample_->position[i] : 0.0);
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
    std::shared_ptr<const Sample> sample;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sample = sample_;
    }
    if (!sample) {
      return hardware_interface::return_type::OK;
    }
    if (active_ && age(sample->received) > timeout_) {
      active_ = false;
      RCLCPP_ERROR(get_logger(), "Swerve feedback timeout");
      return hardware_interface::return_type::ERROR;
    }
    for (size_t i = 0; i < 6; ++i) {
      set_state(names_[i] + "/position", sample->position[i]);
      set_state(names_[i] + "/velocity", sample->velocity[i]);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) {
      return hardware_interface::return_type::OK;
    }
    actuator_msgs::msg::Actuators command;
    for (size_t i = 0; i < 6; ++i) {
      const double value = get_command(names_[i] + (i < 3 ? "/position" : "/velocity"));
      if (!std::isfinite(value)) {
        return hardware_interface::return_type::ERROR;
      }
      if (i < 3) {
        command.position.push_back(value);
      } else {
        command.velocity.push_back(value);
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    command_ = command;
    written_ = SwerveClock::now();
    has_command_ = true;
    return hardware_interface::return_type::OK;
  }

 private:
  static double age(SwerveClock::time_point timestamp)
  {
    return std::chrono::duration<double>(SwerveClock::now() - timestamp).count();
  }

  void receive(sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    const int64_t stamp = int64_t(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
    const double source_age = get_node()->now().seconds() - stamp / 1e9;
    if (stamp <= last_stamp_ || source_age > timeout_ || source_age < -0.1 ||
        msg->name.size() != msg->position.size() || msg->name.size() != msg->velocity.size() ||
        std::unordered_set<std::string>(msg->name.begin(), msg->name.end()).size() !=
            msg->name.size()) {
      return;
    }
    auto sample = std::make_shared<Sample>();
    for (size_t i = 0; i < 6; ++i) {
      const auto found = std::find(msg->name.begin(), msg->name.end(), names_[i]);
      if (found == msg->name.end()) {
        return;
      }
      const size_t index = std::distance(msg->name.begin(), found);
      if (!std::isfinite(msg->position[index]) || !std::isfinite(msg->velocity[index])) {
        return;
      }
      sample->position[i] = msg->position[index];
      sample->velocity[i] = msg->velocity[index];
    }
    sample->received = SwerveClock::now();
    last_stamp_ = stamp;
    std::lock_guard<std::mutex> lock(mutex_);
    sample_ = sample;
  }

  void publish()
  {
    actuator_msgs::msg::Actuators command;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_ || !has_command_ || !sample_ || age(written_) > timeout_ ||
          age(sample_->received) > timeout_) {
        return;
      }
      command = command_;
    }
    command.header.stamp = get_node()->now();
    publisher_->publish(command);
  }

  std::array<std::string, 6> names_;
  std::mutex mutex_;
  std::shared_ptr<const Sample> sample_;
  actuator_msgs::msg::Actuators command_;
  SwerveClock::time_point written_;
  int64_t last_stamp_ = -1;
  double timeout_ = 0.5;
  bool has_command_ = false;
  std::atomic<bool> active_{false};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace coop_carry_hardware
PLUGINLIB_EXPORT_CLASS(coop_carry_hardware::RemoteSwerveSystem, hardware_interface::SystemInterface)
