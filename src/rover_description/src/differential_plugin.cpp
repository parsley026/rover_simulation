// Copyright 2023 Fictionlab sp. z o.o.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components.hh>
#include <gz/plugin/Register.hh>

namespace gazebo_sim_ros2_example
{

class DifferentialPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemUpdate
{
  gz::sim::Model model_{gz::sim::kNullEntity};

  // Swivel differential
  double force_constant_;
  gz::sim::Entity joint_a_, joint_b_;

  // Bogie spring-damper (arm_front / arm_rear joints)
  std::vector<gz::sim::Entity> bogie_joints_;
  double bogie_spring_{50.0};
  double bogie_damping_{5.0};

  bool configured_{false};

public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & /*eventMgr*/) override
  {
    model_ = gz::sim::Model(entity);

    if (!model_.Valid(ecm)) {
      gzerr << "DifferentialPlugin must be attached to a model entity." << std::endl;
      return;
    }

    // --- Swivel differential joints ---
    if (!sdf->HasElement("jointA") || !sdf->HasElement("jointB") ||
      !sdf->HasElement("forceConstant"))
    {
      gzerr << "DifferentialPlugin requires jointA, jointB and forceConstant." << std::endl;
      return;
    }

    force_constant_ = sdf->Get<double>("forceConstant");

    auto init_swivel_joint = [&](const std::string & name) -> gz::sim::Entity {
        auto e = model_.JointByName(ecm, name);
        if (e == gz::sim::kNullEntity) {
          gzerr << "Joint not found: " << name << std::endl;
          return gz::sim::kNullEntity;
        }
        if (!ecm.EntityHasComponentType(e, gz::sim::components::JointPosition().TypeId())) {
          ecm.CreateComponent(e, gz::sim::components::JointPosition());
        }
        if (!ecm.EntityHasComponentType(e, gz::sim::components::JointForceCmd().TypeId())) {
          ecm.CreateComponent(e, gz::sim::components::JointForceCmd({0}));
        }
        return e;
      };

    joint_a_ = init_swivel_joint(sdf->Get<std::string>("jointA"));
    joint_b_ = init_swivel_joint(sdf->Get<std::string>("jointB"));
    if (joint_a_ == gz::sim::kNullEntity || joint_b_ == gz::sim::kNullEntity) {return;}

    // --- Bogie spring/damper joints ---
    if (sdf->HasElement("bogieSpring")) {
      bogie_spring_ = sdf->Get<double>("bogieSpring");
    }
    if (sdf->HasElement("bogieDamping")) {
      bogie_damping_ = sdf->Get<double>("bogieDamping");
    }

    if (sdf->HasElement("bogieJoint")) {
      auto elem = sdf->FindElement("bogieJoint");
      while (elem) {
        auto name = elem->Get<std::string>();
        auto e = model_.JointByName(ecm, name);
        if (e == gz::sim::kNullEntity) {
          gzerr << "Bogie joint not found: " << name << std::endl;
        } else {
          gzmsg << "Bogie joint registered: " << name << std::endl;
          if (!ecm.EntityHasComponentType(e, gz::sim::components::JointPosition().TypeId())) {
            ecm.CreateComponent(e, gz::sim::components::JointPosition());
          }
          if (!ecm.EntityHasComponentType(e, gz::sim::components::JointVelocity().TypeId())) {
            ecm.CreateComponent(e, gz::sim::components::JointVelocity());
          }
          if (!ecm.EntityHasComponentType(e, gz::sim::components::JointForceCmd().TypeId())) {
            ecm.CreateComponent(e, gz::sim::components::JointForceCmd({0}));
          }
          bogie_joints_.push_back(e);
        }
        elem = elem->GetNextElement("bogieJoint");
      }
    }

    configured_ = true;
  }

  void Update(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!configured_ || info.paused) {return;}

    // --- Swivel differential ---
    auto pos_a = ecm.Component<gz::sim::components::JointPosition>(joint_a_)->Data()[0];
    auto pos_b = ecm.Component<gz::sim::components::JointPosition>(joint_b_)->Data()[0];
    auto force_a = ecm.Component<gz::sim::components::JointForceCmd>(joint_a_);
    auto force_b = ecm.Component<gz::sim::components::JointForceCmd>(joint_b_);

    double angle_diff = pos_a - pos_b;
    double stabilize = -(pos_a + pos_b) * force_constant_;

    *force_a = gz::sim::components::JointForceCmd(
      {force_a->Data()[0] - angle_diff * force_constant_ + stabilize});
    *force_b = gz::sim::components::JointForceCmd(
      {force_b->Data()[0] + angle_diff * force_constant_ + stabilize});

    // --- Bogie spring-damper: returns each arm half to neutral (0 rad) ---
    for (auto & e : bogie_joints_) {
      auto pos_c = ecm.Component<gz::sim::components::JointPosition>(e);
      auto vel_c = ecm.Component<gz::sim::components::JointVelocity>(e);
      auto force_c = ecm.Component<gz::sim::components::JointForceCmd>(e);
      if (!pos_c || !vel_c || !force_c) {continue;}

      double pos = pos_c->Data().empty() ? 0.0 : pos_c->Data()[0];
      double vel = vel_c->Data().empty() ? 0.0 : vel_c->Data()[0];

      *force_c = gz::sim::components::JointForceCmd(
        {-bogie_spring_ * pos - bogie_damping_ * vel});
    }
  }
};

}  // namespace gazebo_sim_ros2_example

GZ_ADD_PLUGIN(
  gazebo_sim_ros2_example::DifferentialPlugin,
  gz::sim::System,
  gazebo_sim_ros2_example::DifferentialPlugin::ISystemConfigure,
  gazebo_sim_ros2_example::DifferentialPlugin::ISystemUpdate)