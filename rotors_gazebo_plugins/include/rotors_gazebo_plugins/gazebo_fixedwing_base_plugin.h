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

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rotors_comm/RegisterControlSurface.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

#include "rotors_comm/WindSpeed.h"
#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default topic names
//static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";
static const std::string kDefaultAirSpeedSubTopic = "air_speed";
static const std::string kDefaultCommandSubTopic = "gazebo/command/motor_speed";
static const std::string kDefaultResetModelServiceName = "reset_model";

// Default values for Techpod fixed-wing control surfaces
static constexpr double kDefaultControlSurfaceDeflectionMin = -20.0 * M_PI / 180.0;
static constexpr double kDefaultControlSurfaceDeflectionMax = 20.0 * M_PI / 180.0;
static constexpr int kDefaultAileronChannel = 0;
static constexpr int kDefaultElevatorChannel = 1;
static constexpr int kDefaultRudderChannel = 2;

// Default vehicle parameters for Techpod
static constexpr double kDefaultMass = 2.65;
static constexpr double kDefaultWingSurface = 0.47;
static constexpr double kDefaultWingspan = 2.59;
static constexpr double kDefaultChordLength = 0.18;
static constexpr double kDefaultIThrust = 0.0;
static constexpr double kDefaultInertiaXx = 0.16632;
static constexpr double kDefaultInertiaXy = 0.0;
static constexpr double kDefaultInertiaXz = 0.0755;
static constexpr double kDefaultInertiaYy = 0.3899;
static constexpr double kDefaultInertiaYz = 0.0;
static constexpr double kDefaultInertiaZz = 0.5243;

// Default aerodynamic parameter values for Techpod
static constexpr double kDefaultCD0 = 0.1360;
static constexpr double kDefaultCDa = -0.6737;
static constexpr double kDefaultCDa2 = 5.4546;
static constexpr double kDefaultCL0 = 0.2127;
static constexpr double kDefaultCLa = 10.8060;
static constexpr double kDefaultCLa2 = -46.8324;
static constexpr double kDefaultCLa3 = 60.6017;
static constexpr double kDefaultCLq = 0.0;
static constexpr double kDefaultCLde = 2.5463e-4;
static constexpr double kDefaultCm0 = 0.0435;
static constexpr double kDefaultCma = -2.9690;
static constexpr double kDefaultCmq = -106.1541;
static constexpr double kDefaultCmde = -6.1308;
static constexpr double kDefaultCT0 = 0.0;
static constexpr double kDefaultCT1 = 14.7217;
static constexpr double kDefaultCT2 = 0.0;
static constexpr double kDefaultTauT = 0.0880;
static constexpr double kDefaultClb = -0.0154;
static constexpr double kDefaultClp = -0.1647;
static constexpr double kDefaultClr = 0.0117;
static constexpr double kDefaultClda = 0.0570;
static constexpr double kDefaultCYb = -0.3073;
static constexpr double kDefaultCnb = 0.0430;
static constexpr double kDefaultCnp = -0.0839;
static constexpr double kDefaultCnr = -0.0827;
static constexpr double kDefaultCndr = 0.06;

// Constants
static constexpr double kG = 9.81;
static constexpr double kRhoAir = 1.18;

struct ControlSurface {
  double d_min;
  double d_max;
  double deflection;
};

struct FixedWingAerodynamicParameters {
  double cD0;
  double cDa;
  double cDa2;
  double cL0;
  double cLa;
  double cLa2;
  double cLa3;
  double cLq;
  double cLde;
  double cm0;
  double cma;
  double cmq;
  double cmde;
  double cT0;
  double cT1;
  double cT2;
  double tauT;
  double clb;
  double clp;
  double clr;
  double clda;
  double cYb;
  double cnb;
  double cnp;
  double cnr;
  double cndr;
};

class GazeboFixedWingBasePlugin : public ModelPlugin {
 public:
  GazeboFixedWingBasePlugin();
  virtual ~GazeboFixedWingBasePlugin();

  void ComputeAerodynamicForcesMoments(math::Vector3& forces, math::Vector3& moments);

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;

  ros::NodeHandle* node_handle_;
  //ros::Subscriber wind_speed_sub_;
  ros::Subscriber air_speed_sub_;
  ros::Subscriber command_sub_;
  ros::ServiceServer register_control_surface_service_;
  ros::ServiceServer reset_model_service_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  // Frame names
  std::string cam_parent_frame_;
  std::string cam_child_frame_;

  // Vehicle parameters
  double mass_;
  double wing_surface_;
  double wingspan_;
  double chord_length_;
  double i_thrust_;
  Eigen::Matrix3d inertia_;

  // Pointers to control surfaces joints
  std::vector<physics::JointPtr> ailerons_;
  std::vector<physics::JointPtr> elevators_;
  physics::JointPtr rudder_;

  // Control surfaces info
  ControlSurface aileron_info_;
  ControlSurface elevator_info_;
  ControlSurface rudder_info_;

  // Control surface channels
  int aileron_channel_;
  int elevator_channel_;
  int rudder_channel_;

  double throttle_;

  //math::Vector3 wind_speed_;
  math::Vector3 air_speed_;

  math::Pose start_pose_;

  tf::Transform tf_;
  tf::TransformBroadcaster transform_broadcaster_;

  //void WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed_msg);
  void AirSpeedCallback(const geometry_msgs::TwistStampedConstPtr air_speed_msg);
  void CommandCallback(const mav_msgs::ActuatorsConstPtr& command_msg);

  bool RegisterControlSurfaceCallback(rotors_comm::RegisterControlSurface::Request& req,
                                      rotors_comm::RegisterControlSurface::Response& res);

  bool ResetModelCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  FixedWingAerodynamicParameters aero_params_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_FIXEDWING_BASE_PLUGIN_H
