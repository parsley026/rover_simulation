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

#ifndef ROVER_DESCRIPTION__DIFFERENTIAL_PLUGIN_HPP_
#define ROVER_DESCRIPTION__DIFFERENTIAL_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components.hh>
#include <gz/plugin/Register.hh>
#include <vector>
#include <memory>
#include <string>

namespace gazebo_sim_ros2_example
{

class DifferentialPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  gz::sim::Model model_{gz::sim::kNullEntity};

  // Swivel differential
  double force_constant_{100.0};
  double swivel_damping_{10.0};
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
    gz::sim::EventManager & /*eventMgr*/) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;
};

}  // namespace gazebo_sim_ros2_example

#endif  // ROVER_DESCRIPTION__DIFFERENTIAL_PLUGIN_HPP_
