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

#include "rover_description/DifferentialPlugin.hpp"

namespace gazebo_sim_ros2_example
{

void DifferentialPlugin::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager & /*eventMgr*/)
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

  if (sdf->HasElement("dampingConstant")) {
    swivel_damping_ = sdf->Get<double>("dampingConstant");
  } else if (sdf->HasElement("swivelDamping")) {
    swivel_damping_ = sdf->Get<double>("swivelDamping");
  } else {
    swivel_damping_ = 0.1 * force_constant_;
  }

  auto init_swivel_joint = [&](const std::string & name) -> gz::sim::Entity {
      auto e = model_.JointByName(ecm, name);
      if (e == gz::sim::kNullEntity) {
        gzerr << "Joint not found: " << name << std::endl;
        return gz::sim::kNullEntity;
      }
      if (!ecm.EntityHasComponentType(e, gz::sim::components::JointPosition().TypeId())) {
        ecm.CreateComponent(e, gz::sim::components::JointPosition());
      }
      if (!ecm.EntityHasComponentType(e, gz::sim::components::JointVelocity().TypeId())) {
        ecm.CreateComponent(e, gz::sim::components::JointVelocity());
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

void DifferentialPlugin::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (!configured_ || info.paused) {return;}

  // --- Swivel differential ---
  auto pos_comp_a = ecm.Component<gz::sim::components::JointPosition>(joint_a_);
  auto pos_comp_b = ecm.Component<gz::sim::components::JointPosition>(joint_b_);
  auto vel_comp_a = ecm.Component<gz::sim::components::JointVelocity>(joint_a_);
  auto vel_comp_b = ecm.Component<gz::sim::components::JointVelocity>(joint_b_);
  auto force_a = ecm.Component<gz::sim::components::JointForceCmd>(joint_a_);
  auto force_b = ecm.Component<gz::sim::components::JointForceCmd>(joint_b_);

  if (pos_comp_a && pos_comp_b && force_a && force_b) {
    double pos_a = pos_comp_a->Data().empty() ? 0.0 : pos_comp_a->Data()[0];
    double pos_b = pos_comp_b->Data().empty() ? 0.0 : pos_comp_b->Data()[0];
    double vel_a = (vel_comp_a && !vel_comp_a->Data().empty()) ? vel_comp_a->Data()[0] : 0.0;
    double vel_b = (vel_comp_b && !vel_comp_b->Data().empty()) ? vel_comp_b->Data()[0] : 0.0;

    double stabilize = -(pos_a + pos_b) * force_constant_ - (vel_a + vel_b) * swivel_damping_;

    *force_a = gz::sim::components::JointForceCmd({stabilize});
    *force_b = gz::sim::components::JointForceCmd({stabilize});
  }

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

}  // namespace gazebo_sim_ros2_example

GZ_ADD_PLUGIN(
  gazebo_sim_ros2_example::DifferentialPlugin,
  gz::sim::System,
  gazebo_sim_ros2_example::DifferentialPlugin::ISystemConfigure,
  gazebo_sim_ros2_example::DifferentialPlugin::ISystemPreUpdate)