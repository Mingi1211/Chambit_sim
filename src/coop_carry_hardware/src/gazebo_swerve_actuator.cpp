// 팔 구동과 관절을 공유하지 않는 스워브 서보 및 실제 차체 피드백.
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/odometry.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace coop_carry_hardware
{
class GazeboSwerveActuator : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPreUpdate,
                             public gz::sim::ISystemPostUpdate
{
 public:
  void Configure(const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
                 gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    robot_ = sdf->Get<std::string>("robot");
    base_ = model.LinkByName(ecm, "base_link");
    if (base_ == gz::sim::kNullEntity) {
      throw std::runtime_error("Missing base_link");
    }
    gz::sim::Link(base_).EnableVelocityChecks(ecm);
    for (const auto & tag : {"steering_joint", "drive_joint"}) {
      auto element = sdf->FindElement(tag);
      while (element) {
        auto joint = model.JointByName(ecm, element->Get<std::string>());
        if (joint == gz::sim::kNullEntity) {
          throw std::runtime_error("Missing swerve joint");
        }
        (std::string(tag) == "steering_joint" ? steering_ : wheels_).push_back(joint);
        if (!ecm.Component<gz::sim::components::JointPosition>(joint)) {
          ecm.CreateComponent(joint, gz::sim::components::JointPosition());
        }
        if (!ecm.Component<gz::sim::components::JointVelocity>(joint)) {
          ecm.CreateComponent(joint, gz::sim::components::JointVelocity());
        }
        ecm.CreateComponent(joint, gz::sim::components::JointPositionReset({0.0}));
        element = element->GetNextElement(tag);
      }
    }
    if (steering_.size() != 3 || wheels_.size() != 3) {
      throw std::runtime_error("Three steering and three drive joints required");
    }
    hold_.assign(3, 0.0);
    timeout_ = sdf->Get<double>("command_timeout", 0.3).first;
    gain_ = sdf->Get<double>("steering_gain", 15.0).first;
    feedback_ = node_.Advertise<gz::msgs::Odometry>(sdf->Get<std::string>("feedback_topic"));
    node_.Subscribe(sdf->Get<std::string>("command_topic"), &GazeboSwerveActuator::OnCommand, this);
  }

  void OnCommand(const gz::msgs::Actuators & msg)
  {
    if (msg.position_size() != 3 || msg.velocity_size() != 3) {
      return;
    }
    for (double value : msg.position()) {
      if (!std::isfinite(value)) {
        return;
      }
    }
    for (double value : msg.velocity()) {
      if (!std::isfinite(value)) {
        return;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const double stamp = msg.header().stamp().sec() + msg.header().stamp().nsec() / 1e9;
    if (stamp <= stamp_) {
      return;
    }
    positions_.assign(msg.position().begin(), msg.position().end());
    velocities_.assign(msg.velocity().begin(), msg.velocity().end());
    stamp_ = stamp;
    received_ = std::chrono::steady_clock::now();
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    std::vector<double> positions, velocities;
    bool live;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double source_age = std::chrono::duration<double>(info.simTime).count() - stamp_;
      live = positions_.size() == 3 && source_age >= -0.1 && source_age < timeout_ &&
             std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() <
                 timeout_;
      positions = positions_;
      velocities = velocities_;
    }
    for (size_t i = 0; i < 3; ++i) {
      const auto position = ecm.Component<gz::sim::components::JointPosition>(steering_[i]);
      if (position && !position->Data().empty()) {
        const double measured = position->Data()[0];
        if (was_live_ && !live) {
          hold_[i] = measured;
        }
        const double target = live ? std::clamp(positions[i], -6.28, 6.28) : hold_[i];
        SetVelocity(ecm, steering_[i], std::clamp(gain_ * (target - measured), -2.0, 2.0));
      }
      // 타임아웃은 상위 Python/ROS 프로세스와 독립적으로 구동 속도를 0으로 만든다.
      SetVelocity(ecm, wheels_[i], live ? std::clamp(velocities[i], -20.0, 20.0) : 0.0);
    }
    was_live_ = live;
  }

  void PostUpdate(const gz::sim::UpdateInfo & info,
                  const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || info.simTime - published_ < std::chrono::milliseconds(20)) {
      return;
    }
    gz::sim::Link link(base_);
    auto linear = link.WorldLinearVelocity(ecm);
    auto angular = link.WorldAngularVelocity(ecm);
    if (!linear || !angular) {
      return;
    }
    const auto pose = gz::sim::worldPose(base_, ecm);
    gz::msgs::Odometry msg;
    auto stamp = msg.mutable_header()->mutable_stamp();
    const auto nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    stamp->set_sec(nanoseconds / 1000000000LL);
    stamp->set_nsec(nanoseconds % 1000000000LL);
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value("world");
    auto child = msg.mutable_header()->add_data();
    child->set_key("child_frame_id");
    child->add_value(robot_ + "/base_link");
    gz::msgs::Set(msg.mutable_pose(), pose);
    gz::msgs::Set(msg.mutable_twist()->mutable_linear(), pose.Rot().RotateVectorReverse(*linear));
    gz::msgs::Set(msg.mutable_twist()->mutable_angular(), pose.Rot().RotateVectorReverse(*angular));
    feedback_.Publish(msg);
    published_ = info.simTime;
  }

 private:
  static void SetVelocity(gz::sim::EntityComponentManager & ecm, gz::sim::Entity joint,
                          double value)
  {
    auto command = ecm.Component<gz::sim::components::JointVelocityCmd>(joint);
    if (command) {
      command->Data() = {value};
    } else {
      ecm.CreateComponent(joint, gz::sim::components::JointVelocityCmd({value}));
    }
  }
  gz::transport::Node node_;
  gz::transport::Node::Publisher feedback_;
  gz::sim::Entity base_ = gz::sim::kNullEntity;
  std::vector<gz::sim::Entity> steering_, wheels_;
  std::vector<double> positions_, velocities_, hold_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_;
  std::chrono::steady_clock::duration published_{};
  std::string robot_;
  double stamp_ = -1.0, timeout_ = 0.3, gain_ = 15.0;
  bool was_live_ = false;
};
}  // namespace coop_carry_hardware
GZ_ADD_PLUGIN(coop_carry_hardware::GazeboSwerveActuator, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate, gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(coop_carry_hardware::GazeboSwerveActuator,
                    "coop_carry_hardware::GazeboSwerveActuator")
