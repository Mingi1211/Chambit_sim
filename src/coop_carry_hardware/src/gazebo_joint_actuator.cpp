#include <gz/msgs/actuators.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/transport/Node.hh>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace coop_carry_hardware
{
// Gazebo 내부의 가상 관절 서보. 위치 오차를 속도 명령으로 변환한다.
// 궤적 계산과 Controller Manager는 로봇 컨테이너에서 실행한다.
class GazeboJointActuator : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate
{
  struct Joint
  {
    gz::sim::Entity entity;
    double initial;
    double lower;
    double upper;
    double velocity;
    double hold;
    int command_index;
    int mimic_index;
  };

 public:
  void Configure(const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
                 gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    timeout_ = sdf->Get<double>("command_timeout", 0.5).first;
    gain_ = sdf->Get<double>("position_gain", 10.0).first;
    auto element = sdf->FindElement("joint");
    while (element) {
      const auto name = element->Get<std::string>("name");
      auto joint = model.JointByName(ecm, name);
      if (joint == gz::sim::kNullEntity) {
        throw std::runtime_error("Missing joint " + name);
      }
      const double initial = element->Get<double>("initial");
      joints_.push_back({joint, initial, element->Get<double>("lower"),
                         element->Get<double>("upper"), element->Get<double>("velocity"), initial,
                         element->Get<int>("command_index"), element->Get<int>("mimic_index")});
      if (!ecm.Component<gz::sim::components::JointPosition>(joint)) {
        ecm.CreateComponent(joint, gz::sim::components::JointPosition());
      }
      ecm.CreateComponent(joint, gz::sim::components::JointPositionReset({initial}));
      element = element->GetNextElement("joint");
    }
    count_ = sdf->Get<unsigned int>("command_count");
    node_.Subscribe(sdf->Get<std::string>("command_topic"), &GazeboJointActuator::OnCommand, this);
  }

  void OnCommand(const gz::msgs::Actuators & msg)
  {
    if (msg.position_size() != static_cast<int>(count_)) {
      return;
    }
    for (double value : msg.position()) {
      if (!std::isfinite(value)) {
        return;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    command_.assign(msg.position().begin(), msg.position().end());
    command_stamp_ = msg.header().stamp().sec() + msg.header().stamp().nsec() / 1e9;
    received_ = std::chrono::steady_clock::now();
  }

  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }
    std::vector<double> command;
    bool live;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      live = command_.size() == count_ &&
             std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() <
                 timeout_ &&
             std::chrono::duration<double>(info.simTime).count() - command_stamp_ < timeout_ &&
             std::chrono::duration<double>(info.simTime).count() - command_stamp_ > -0.1;
      command = command_;
    }
    if (was_live_ && !live) {
      hold_current_positions(ecm);
    }
    was_live_ = live;
    apply_joint_targets(ecm, command, live);
  }

 private:
  // 명령이 끊긴 시점의 측정 위치를 유지 목표로 저장한다.
  void hold_current_positions(gz::sim::EntityComponentManager & ecm)
  {
    for (auto & joint : joints_) {
      const auto position = ecm.Component<gz::sim::components::JointPosition>(joint.entity);
      if (position && !position->Data().empty()) {
        joint.hold = position->Data()[0];
      }
    }
  }

  void apply_joint_targets(gz::sim::EntityComponentManager & ecm,
                           const std::vector<double> & command, bool live)
  {
    for (auto & joint : joints_) {
      auto position = ecm.Component<gz::sim::components::JointPosition>(joint.entity);
      if (!position || position->Data().empty()) {
        continue;
      }
      double target = joint.hold;
      if (live && joint.command_index >= 0) {
        target = command[joint.command_index];
      }
      if (joint.mimic_index >= 0) {
        auto source =
            ecm.Component<gz::sim::components::JointPosition>(joints_[joint.mimic_index].entity);
        if (source && !source->Data().empty()) {
          target = source->Data()[0];
        }
      }
      // 목표 위치와 속도를 제한한 뒤 Gazebo 물리 엔진에 전달한다.
      target = std::clamp(target, joint.lower, joint.upper);
      const double velocity =
          std::clamp(gain_ * (target - position->Data()[0]), -joint.velocity, joint.velocity);
      auto output = ecm.Component<gz::sim::components::JointVelocityCmd>(joint.entity);
      if (!output) {
        ecm.CreateComponent(joint.entity, gz::sim::components::JointVelocityCmd({velocity}));
      } else {
        output->Data() = {velocity};
      }
    }
  }

  gz::transport::Node node_;
  std::vector<Joint> joints_;
  std::vector<double> command_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point received_;
  unsigned int count_ = 0;
  double timeout_ = 0.5;
  double gain_ = 10.0;
  double command_stamp_ = 0.0;
  bool was_live_ = false;
};
}  // namespace coop_carry_hardware
GZ_ADD_PLUGIN(coop_carry_hardware::GazeboJointActuator, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(coop_carry_hardware::GazeboJointActuator,
                    "coop_carry_hardware::GazeboJointActuator")
