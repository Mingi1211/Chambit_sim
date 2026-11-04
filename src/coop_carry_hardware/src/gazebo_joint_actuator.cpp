#include <gz/msgs/actuators.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointForceCmd.hh>
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
    // [문서 밖] 김민기: 토크 경로 상태
    bool effort_active = false;
    bool release_hold = false;
    double command_at_release = 0.0;
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
    // [문서 밖] effort(토크) 경로 — 김민기 하위 제어. effort_topic이 없으면 기존 위치 서보만 동작(현대 기본 동작 그대로)
    if (sdf->HasElement("effort_topic")) {
      effort_timeout_ = sdf->Get<double>("effort_timeout", 0.1).first;
      effort_limit_ = sdf->Get<double>("effort_limit", 60.0).first;
      node_.Subscribe(sdf->Get<std::string>("effort_topic"), &GazeboJointActuator::OnEffort, this);
    }
  }

  // 토크 명령: Actuators.normalized[command_index] = 토크[N·m]. NaN인 관절은 기존 위치 서보 유지.
  // position[i]이 숫자면 그 관절(그리퍼) 위치 목표를 덮어씀 — 그리퍼 열기·닫기
  void OnEffort(const gz::msgs::Actuators & msg)
  {
    if (msg.normalized_size() != static_cast<int>(count_)) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    effort_.assign(msg.normalized().begin(), msg.normalized().end());
    override_.assign(count_, std::nan(""));
    if (msg.position_size() == static_cast<int>(count_)) {
      override_.assign(msg.position().begin(), msg.position().end());
    }
    effort_received_ = std::chrono::steady_clock::now();
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
    std::vector<double> command, effort, override;
    bool live, effort_live;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      live = command_.size() == count_ &&
             std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() <
                 timeout_ &&
             std::chrono::duration<double>(info.simTime).count() - command_stamp_ < timeout_ &&
             std::chrono::duration<double>(info.simTime).count() - command_stamp_ > -0.1;
      command = command_;
      effort_live = effort_.size() == count_ &&
                    std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                  effort_received_).count() < effort_timeout_;
      effort = effort_;
      override = override_;
    }
    if (was_live_ && !live) {
      hold_current_positions(ecm);
    }
    was_live_ = live;
    apply_joint_targets(ecm, command, live, effort, override, effort_live);
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
                           const std::vector<double> & command, bool live,
                           const std::vector<double> & effort,
                           const std::vector<double> & override, bool effort_live)
  {
    for (auto & joint : joints_) {
      auto position = ecm.Component<gz::sim::components::JointPosition>(joint.entity);
      if (!position || position->Data().empty()) {
        continue;
      }
      // [문서 밖] effort 관절: 속도 명령을 지우고(남아 있으면 물리엔진이 속도를 강제) 토크만 건다
      if (effort_live && joint.command_index >= 0 && std::isfinite(effort[joint.command_index])) {
        ecm.RemoveComponent<gz::sim::components::JointVelocityCmd>(joint.entity);
        const double tau = std::clamp(effort[joint.command_index], -effort_limit_, effort_limit_);
        auto force = ecm.Component<gz::sim::components::JointForceCmd>(joint.entity);
        if (!force) {
          ecm.CreateComponent(joint.entity, gz::sim::components::JointForceCmd({tau}));
        } else {
          force->Data() = {tau};
        }
        joint.effort_active = true;
        continue;
      }
      if (joint.effort_active) {
        // 토크가 끊기면(노드 종료·지연) 그 순간 자세를 위치 hold — 팔이 떨어지지 않게(안전).
        // 이후 상위 위치 명령이 실제로 바뀌기 전까지는 예전 위치 명령으로 튀지 않는다
        joint.effort_active = false;
        joint.release_hold = true;
        ecm.RemoveComponent<gz::sim::components::JointForceCmd>(joint.entity);
        joint.hold = position->Data()[0];
        joint.command_at_release =
            (live && joint.command_index >= 0) ? command[joint.command_index] : 0.0;
      }
      if (joint.release_hold && live && joint.command_index >= 0 &&
          std::abs(command[joint.command_index] - joint.command_at_release) > 1e-3) {
        joint.release_hold = false;
      }
      double target = joint.hold;
      if (live && joint.command_index >= 0 && !joint.release_hold) {
        target = command[joint.command_index];
      }
      if (effort_live && joint.command_index >= 0 && std::isfinite(override[joint.command_index])) {
        target = override[joint.command_index];
        joint.hold = target;
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
  std::vector<double> effort_, override_;
  std::chrono::steady_clock::time_point effort_received_;
  double effort_timeout_ = 0.1;
  double effort_limit_ = 60.0;
};
}  // namespace coop_carry_hardware
GZ_ADD_PLUGIN(coop_carry_hardware::GazeboJointActuator, gz::sim::System, gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(coop_carry_hardware::GazeboJointActuator,
                    "coop_carry_hardware::GazeboJointActuator")
