/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_GAZEBO_PLUGINS_CONTROL_SURFACE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_CONTROL_SURFACE_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <rotors_comm/RegisterControlSurface.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static constexpr double kDefaultAngleMin = -0.349066;
static constexpr double kDefaultAngleMax = 0.349066;

class GazeboControlSurfacePlugin : public ModelPlugin {
 public:

  GazeboControlSurfacePlugin();
  virtual ~GazeboControlSurfacePlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

 private:
  // ROS interface
  std::string namespace_;
  ros::NodeHandle* node_handle_;
  ros::ServiceClient register_control_surface_client_;

  // Control surface parameters
  double angle_min_;
  double angle_max_;
  std::string joint_name_;
  std::string surface_type_;
  std::string surface_side_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_CONTROL_SURFACE_PLUGIN_H
