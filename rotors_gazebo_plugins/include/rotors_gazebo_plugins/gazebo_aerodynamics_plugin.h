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

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_AERODYNAMICS_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_AERODYNAMICS_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/TwistStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <ros/ros.h>

#include "rotors_comm/RegisterControlSurface.h"
#include "rotors_comm/WindSpeed.h"
#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default interface values
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";
static const std::string kDefaultAirSpeedSubTopic = "air_speed";
static const std::string kDefaultCommandSubTopic = "gazebo/command/motor_speed";
static constexpr bool kDefaultJoyInput = false;

// Default values for fixed-wing control surfaces (Techpod model)
static constexpr double kDefaultControlSurfaceDeflectionMin = -20.0 * M_PI / 180.0;
static constexpr double kDefaultControlSurfaceDeflectionMax = 20.0 * M_PI / 180.0;

// Default vehicle parameters (Techpod model)
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

// Default aerodynamic parameter values (Techpod model)
static const Eigen::Vector3d kDefaultCDAlpha = Eigen::Vector3d(0.1360, -0.6737, 5.4546);
static const Eigen::Vector3d kDefaultCDBeta = Eigen::Vector3d(0.0195, 0.0, -0.3842);
static const Eigen::Vector3d kDefaultCDDeltaAil = Eigen::Vector3d(0.0195, 1.4205e-4, 7.5037e-6);
static const Eigen::Vector3d kDefaultCDDeltaFlp = Eigen::Vector3d(0.0195, 2.7395e-4, 1.23e-5);

static const Eigen::Vector2d kDefaultCYBeta = Eigen::Vector2d(0.0, -0.3073);

static const Eigen::Vector4d kDefaultCLAlpha = Eigen::Vector4d(0.2127, 10.8060, -46.8324, 60.6017);
static const Eigen::Vector2d kDefaultCLDeltaAil = Eigen::Vector2d(0.3304, 0.0048);
static const Eigen::Vector2d kDefaultCLDeltaFlp = Eigen::Vector2d(0.3304, 0.0073);

static const Eigen::Vector2d kDefaultCLmBeta = Eigen::Vector2d(0.0, -0.0154);
static const Eigen::Vector2d kDefaultCLmP = Eigen::Vector2d(0.0, -0.1647);
static const Eigen::Vector2d kDefaultCLmR = Eigen::Vector2d(0.0, 0.0117);
static const Eigen::Vector2d kDefaultCLmDeltaAil = Eigen::Vector2d(0.0, 0.0570);
static const Eigen::Vector2d kDefaultCLmDeltaFlp = Eigen::Vector2d(0.0, 0.001);

static const Eigen::Vector2d kDefaultCMmAlpha = Eigen::Vector2d(0.0435, -2.9690);
static const Eigen::Vector2d kDefaultCMmQ = Eigen::Vector2d(-0.1173, -106.1541);
static const Eigen::Vector2d kDefaultCMmDeltaElv = Eigen::Vector2d(-0.1173, -6.1308);

static const Eigen::Vector2d kDefaultCNmBeta = Eigen::Vector2d(0.0, 0.0430);
static const Eigen::Vector2d kDefaultCNmR = Eigen::Vector2d(0.0, -0.0827);
static const Eigen::Vector2d kDefaultCNmDeltaRud = Eigen::Vector2d(0.0, 0.06);

static const Eigen::Vector3d kDefaultCT = Eigen::Vector3d(0.0, 14.7217, 0.0);

// Constants
static constexpr double kG = 9.81;
static constexpr double kRhoAir = 1.18;

struct ControlSurface {
  double d_min;
  double d_max;
  double deflection;
};

struct FixedWingAerodynamicParameters {
  Eigen::Vector3d c_D_alpha;
  Eigen::Vector3d c_D_beta;
  Eigen::Vector3d c_D_delta_ail;
  Eigen::Vector3d c_D_delta_flp;

  Eigen::Vector2d c_Y_beta;

  Eigen::Vector4d c_L_alpha;
  Eigen::Vector2d c_L_delta_ail;
  Eigen::Vector2d c_L_delta_flp;

  Eigen::Vector2d c_Lm_beta;
  Eigen::Vector2d c_Lm_p;
  Eigen::Vector2d c_Lm_r;
  Eigen::Vector2d c_Lm_delta_ail;
  Eigen::Vector2d c_Lm_delta_flp;

  Eigen::Vector2d c_Mm_alpha;
  Eigen::Vector2d c_Mm_q;
  Eigen::Vector2d c_Mm_delta_elv;

  Eigen::Vector2d c_Nm_beta;
  Eigen::Vector2d c_Nm_r;
  Eigen::Vector2d c_Nm_delta_rud;

  Eigen::Vector3d c_T;
};

class GazeboAerodynamicsPlugin : public ModelPlugin {
 public:
  GazeboAerodynamicsPlugin();
  virtual ~GazeboAerodynamicsPlugin();

  void ComputeAerodynamicForcesMoments(math::Vector3& forces, math::Vector3& moments);

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;

  ros::NodeHandle* node_handle_;
  ros::Subscriber wind_speed_sub_;
  ros::Subscriber air_speed_sub_;
  ros::Subscriber command_sub_;
  ros::Subscriber roll_pitch_yawrate_thrust_sub_;
  ros::ServiceServer register_control_surface_service_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

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
  std::vector<physics::JointPtr> flaps_;
  physics::JointPtr rudder_;

  // Control surfaces info
  ControlSurface aileron_left_info_;
  ControlSurface aileron_right_info_;
  ControlSurface elevator_info_;
  ControlSurface rudder_info_;
  ControlSurface flap_info_;

  // Control surface channels
  int aileron_channel_;
  int elevator_channel_;
  int rudder_channel_;

  double throttle_;

  bool joy_input_;

  math::Vector3 wind_speed_;
  math::Vector3 air_speed_;

  void WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed_msg);
  void AirSpeedCallback(const geometry_msgs::TwistStampedConstPtr air_speed_msg);
  void CommandCallback(const mav_msgs::ActuatorsConstPtr& command_msg);
  void RollPitchYawrateThrustCallback(
      const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference_msg);

  bool RegisterControlSurfaceCallback(rotors_comm::RegisterControlSurface::Request& req,
                                      rotors_comm::RegisterControlSurface::Response& res);

  FixedWingAerodynamicParameters aero_params_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_AERODYNAMICS_PLUGIN_H
