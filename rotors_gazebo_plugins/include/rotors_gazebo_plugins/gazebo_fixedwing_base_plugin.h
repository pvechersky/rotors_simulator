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
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/RegisterAeroSurface.h"

namespace gazebo {
// Default values
static const std::string kDefaultCommandSubTopic = "gazebo/command/motor_speed";
static const std::string kDefaultResetSubTopic = "reset";
static constexpr double kDefaultAirDensity = 1.225;
static constexpr double kDefaultAlphaStall = 0.3;

// Constants
static constexpr double kCL0 = 0.2127;
static constexpr double kCLa = 10.8060;
static constexpr double kCLa2 = -46.8324;
static constexpr double kCLa3 = 60.6017;

static constexpr double kCD0 = 0.1360;
static constexpr double kCDa = -0.6737;
static constexpr double kCDa2 = 5.4546;

static constexpr double kCYb = -0.3073;

static constexpr double kClb = -0.0154;
static constexpr double kClp = -0.1647;
static constexpr double kClr = 0.0117;
static constexpr double kClda = 0.0570;

static constexpr double kCm0 = 0.0435;
static constexpr double kCma = -2.9690;
static constexpr double kCmq = -106.1541;
static constexpr double kCmde = -6.1308;

static constexpr double kCnb = 0.0430;
static constexpr double kCnp = -0.0839;
static constexpr double kCnr = -0.0827;
static constexpr double kCndr = 0.06;

static constexpr double kBWing = 2.59;
static constexpr double kCChord = 0.18;
static constexpr double kRhoAir = 1.18;
static constexpr double kSWing = 0.47;

// Temporary
static constexpr double kDeflectionMin = -20.0 * M_PI / 180.0;
static constexpr double kDeflectionMax = 20.0 * M_PI / 180.0;

class GazeboFixedWingBasePlugin : public ModelPlugin {
 public:
  GazeboFixedWingBasePlugin();
  virtual ~GazeboFixedWingBasePlugin();

  bool RegisterAeroSurface(rotors_gazebo_plugins::RegisterAeroSurface::Request& req,
                           rotors_gazebo_plugins::RegisterAeroSurface::Response& res);

  void ComputeAerodynamicForcesAndMoments(math::Vector3& forces, math::Vector3& moments);

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;

  ros::NodeHandle* node_handle_;
  ros::ServiceServer register_aero_surface_service_;
  ros::Subscriber deflections_sub_;
  ros::Subscriber reset_sub_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  // Pointers to the control surfaces joints
  std::vector<physics::JointPtr> ailerons_;
  std::vector<physics::JointPtr> elevators_;
  physics::JointPtr rudder_;

  double air_density_;
  double alpha_stall_;
  double total_wing_area_;
  double total_tail_area_;

  double aileron_deflection_;
  double elevator_deflection_;
  double rudder_deflection_;

  math::Quaternion orientation_;

  void DeflectionsCallback(const mav_msgs::ActuatorsConstPtr& deflections);
  void ResetCallback(const std_msgs::BoolConstPtr& reset_msg);
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_FIXEDWING_BASE_PLUGIN_H
