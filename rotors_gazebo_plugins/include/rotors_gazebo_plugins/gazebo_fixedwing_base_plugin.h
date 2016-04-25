/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_FIXEDWING_BASE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_FIXEDWING_BASE_PLUGIN_H

#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <std_msgs/Bool.h>

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/RegisterAeroSurface.h"

namespace gazebo {
// Default values
static const std::string kDefaultResetSubTopic = "reset";
static constexpr double kDefaultAirDensity = 1.225;
static constexpr double kDefaultAlphaStall = 0.3;

class GazeboFixedWingBasePlugin : public ModelPlugin {
 public:
  GazeboFixedWingBasePlugin();
  virtual ~GazeboFixedWingBasePlugin();

  bool RegisterAeroSurface(rotors_gazebo_plugins::RegisterAeroSurface::Request& req,
                           rotors_gazebo_plugins::RegisterAeroSurface::Response& res);

  math::Vector3 ComputeAerodynamicForces(math::Vector3& vel);
  math::Vector3 ComputeAerodynamicMoments(math::Vector3& vel);

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;

  ros::NodeHandle* node_handle_;
  ros::ServiceServer register_aero_surface_service_;
  ros::Subscriber reset_sub_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  double air_density_;
  double alpha_stall_;
  double total_wing_area_;

  math::Quaternion orientation_;

  void resetCallback(const std_msgs::BoolConstPtr& reset_msg);
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_FIXEDWING_BASE_PLUGIN_H
