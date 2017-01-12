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

#include "rotors_gazebo_plugins/gazebo_aerodynamics_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboAerodynamicsPlugin::GazeboAerodynamicsPlugin()
    : ModelPlugin(),
      node_handle_(0),
      throttle_(0.0) {
}

GazeboAerodynamicsPlugin::~GazeboAerodynamicsPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboAerodynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model and the world
  model_ = _model;
  world_ = model_->GetWorld();

  // Get the robot namespace
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_aerodynamics_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  // Get the link name
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_aerodynamics_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_aerodynamics_plugin] Couldn't find specified link \"" << link_name << "\".");

  std::string wind_speed_sub_topic;
  std::string air_speed_sub_topic;
  std::string command_sub_topic;
  std::string roll_pitch_yawrate_thrust_sub_topic;
  getSdfParam<bool>(_sdf, "joyInput", joy_input_, kDefaultJoyInput);
  getSdfParam<std::string>(_sdf, "windSpeedSubTopic",
                           wind_speed_sub_topic,
                           kDefaultWindSpeedSubTopic);
  getSdfParam<std::string>(_sdf, "airSpeedSubTopic",
                           air_speed_sub_topic,
                           kDefaultAirSpeedSubTopic);
  getSdfParam<std::string>(_sdf, "commandSubTopic",
                           command_sub_topic,
                           kDefaultCommandSubTopic);
  getSdfParam<std::string>(_sdf, "rollPitchYawrate,ThrustSubTopic",
                           roll_pitch_yawrate_thrust_sub_topic,
                           mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST);

  ros::NodeHandle pnh("~");

  // Read the vehicle parameters from rosparam
  pnh.param<double>("mass", mass_, kDefaultMass);
  pnh.param<double>("wing_surface", wing_surface_, kDefaultWingSurface);
  pnh.param<double>("wingspan", wingspan_, kDefaultWingspan);
  pnh.param<double>("chord_length", chord_length_, kDefaultChordLength);
  pnh.param<double>("i_thrust", i_thrust_, kDefaultIThrust);
  pnh.param<double>("inertia/xx", inertia_(0, 0), kDefaultInertiaXx);
  pnh.param<double>("inertia/xy", inertia_(0, 1), kDefaultInertiaXy);
  inertia_(1, 0) = inertia_(0, 1);
  pnh.param<double>("inertia/xz", inertia_(0, 2), kDefaultInertiaXz);
  inertia_(2, 0) = inertia_(0, 2);
  pnh.param<double>("inertia/yy", inertia_(1, 1), kDefaultInertiaYy);
  pnh.param<double>("inertia/yz", inertia_(1, 2), kDefaultInertiaYz);
  inertia_(2, 1) = inertia_(1, 2);
  pnh.param<double>("inertia/zz", inertia_(2, 2), kDefaultInertiaZz);

  // Read the aerodynamic parameters from rosparam
  pnh.param<double>("cD_alpha_0", aero_params_.c_D_alpha(0), kDefaultCDAlpha(0));
  pnh.param<double>("cD_alpha_1", aero_params_.c_D_alpha(1), kDefaultCDAlpha(1));
  pnh.param<double>("cD_alpha_2", aero_params_.c_D_alpha(2), kDefaultCDAlpha(2));
  pnh.param<double>("cD_beta_0", aero_params_.c_D_beta(0), kDefaultCDBeta(0));
  pnh.param<double>("cD_beta_1", aero_params_.c_D_beta(1), kDefaultCDBeta(1));
  pnh.param<double>("cD_beta_2", aero_params_.c_D_beta(2), kDefaultCDBeta(2));
  pnh.param<double>("cD_delta_ail_0", aero_params_.c_D_delta_ail(0), kDefaultCDDeltaAil(0));
  pnh.param<double>("cD_delta_ail_1", aero_params_.c_D_delta_ail(1), kDefaultCDDeltaAil(1));
  pnh.param<double>("cD_delta_ail_2", aero_params_.c_D_delta_ail(2), kDefaultCDDeltaAil(2));
  pnh.param<double>("cD_delta_flp_0", aero_params_.c_D_delta_flp(0), kDefaultCDDeltaFlp(0));
  pnh.param<double>("cD_delta_flp_1", aero_params_.c_D_delta_flp(1), kDefaultCDDeltaFlp(1));
  pnh.param<double>("cD_delta_flp_2", aero_params_.c_D_delta_flp(2), kDefaultCDDeltaFlp(2));

  pnh.param<double>("cY_beta_0", aero_params_.c_Y_beta(0), kDefaultCYBeta(0));
  pnh.param<double>("cY_beta_1", aero_params_.c_Y_beta(1), kDefaultCYBeta(1));

  pnh.param<double>("cL_alpha_0", aero_params_.c_L_alpha(0), kDefaultCLAlpha(0));
  pnh.param<double>("cL_alpha_1", aero_params_.c_L_alpha(1), kDefaultCLAlpha(1));
  pnh.param<double>("cL_alpha_2", aero_params_.c_L_alpha(2), kDefaultCLAlpha(2));
  pnh.param<double>("cL_alpha_3", aero_params_.c_L_alpha(3), kDefaultCLAlpha(3));
  pnh.param<double>("cL_delta_ail_0", aero_params_.c_L_delta_ail(0), kDefaultCLDeltaAil(0));
  pnh.param<double>("cL_delta_ail_1", aero_params_.c_L_delta_ail(1), kDefaultCLDeltaAil(1));
  pnh.param<double>("cL_delta_flp_0", aero_params_.c_L_delta_flp(0), kDefaultCLDeltaFlp(0));
  pnh.param<double>("cL_delta_flp_1", aero_params_.c_L_delta_flp(1), kDefaultCLDeltaFlp(1));

  pnh.param<double>("cLm_beta_0", aero_params_.c_Lm_beta(0), kDefaultCLmBeta(0));
  pnh.param<double>("cLm_beta_1", aero_params_.c_Lm_beta(1), kDefaultCLmBeta(1));
  pnh.param<double>("cLm_p_0", aero_params_.c_Lm_p(0), kDefaultCLmP(0));
  pnh.param<double>("cLm_p_1", aero_params_.c_Lm_p(1), kDefaultCLmP(1));
  pnh.param<double>("cLm_r_0", aero_params_.c_Lm_r(0), kDefaultCLmR(0));
  pnh.param<double>("cLm_r_1", aero_params_.c_Lm_r(1), kDefaultCLmR(1));
  pnh.param<double>("cLm_delta_ail_0", aero_params_.c_Lm_delta_ail(0), kDefaultCLmDeltaAil(0));
  pnh.param<double>("cLm_delta_ail_1", aero_params_.c_Lm_delta_ail(1), kDefaultCLmDeltaAil(1));
  pnh.param<double>("cLm_delta_flp_0", aero_params_.c_Lm_delta_flp(0), kDefaultCLmDeltaFlp(0));
  pnh.param<double>("cLm_delta_flp_1", aero_params_.c_Lm_delta_flp(1), kDefaultCLmDeltaFlp(1));

  pnh.param<double>("cMm_alpha_0", aero_params_.c_Mm_alpha(0), kDefaultCMmAlpha(0));
  pnh.param<double>("cMm_alpha_1", aero_params_.c_Mm_alpha(1), kDefaultCMmAlpha(1));
  pnh.param<double>("cMm_q_0", aero_params_.c_Mm_q(0), kDefaultCMmQ(0));
  pnh.param<double>("cMm_q_1", aero_params_.c_Mm_q(1), kDefaultCMmQ(1));
  pnh.param<double>("cMm_delta_elv_0", aero_params_.c_Mm_delta_elv(0), kDefaultCMmDeltaElv(0));
  pnh.param<double>("cMm_delta_elv_1", aero_params_.c_Mm_delta_elv(1), kDefaultCMmDeltaElv(1));

  pnh.param<double>("cNm_beta_0", aero_params_.c_Nm_beta(0), kDefaultCNmBeta(0));
  pnh.param<double>("cNm_beta_1", aero_params_.c_Nm_beta(1), kDefaultCNmBeta(1));
  pnh.param<double>("cNm_r_0", aero_params_.c_Nm_r(0), kDefaultCNmR(0));
  pnh.param<double>("cNm_r_1", aero_params_.c_Nm_r(1), kDefaultCNmR(1));
  pnh.param<double>("cNm_delta_rud_0", aero_params_.c_Nm_delta_rud(0), kDefaultCNmDeltaRud(0));
  pnh.param<double>("cNm_delta_rud_1", aero_params_.c_Nm_delta_rud(1), kDefaultCNmDeltaRud(1));

  pnh.param<double>("cT_0", aero_params_.c_T(0), kDefaultCT(0));
  pnh.param<double>("cT_1", aero_params_.c_T(1), kDefaultCT(1));
  pnh.param<double>("cT_2", aero_params_.c_T(2), kDefaultCT(2));

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboAerodynamicsPlugin::OnUpdate, this, _1));

  wind_speed_sub_ = node_handle_->subscribe(wind_speed_sub_topic, 1, &GazeboAerodynamicsPlugin::WindSpeedCallback, this);
  air_speed_sub_ = node_handle_->subscribe(air_speed_sub_topic, 1,&GazeboAerodynamicsPlugin::AirSpeedCallback, this);

  if (joy_input_) {
    roll_pitch_yawrate_thrust_sub_ =
            node_handle_->subscribe(roll_pitch_yawrate_thrust_sub_topic, 1, &GazeboAerodynamicsPlugin::RollPitchYawrateThrustCallback, this);
  }
  else {
    command_sub_ =
            node_handle_->subscribe(command_sub_topic, 1, &GazeboAerodynamicsPlugin::CommandCallback, this);
  }

  register_control_surface_service_ =
          node_handle_->advertiseService("register_control_surface",
                                         &GazeboAerodynamicsPlugin::RegisterControlSurfaceCallback,
                                         this);
}

void GazeboAerodynamicsPlugin::OnUpdate(const common::UpdateInfo& _info) {
  math::Vector3 forces;
  math::Vector3 moments;

  ComputeAerodynamicForcesMoments(forces, moments);

  //link_->AddLinkForce(forces);
  link_->AddRelativeForce(forces);
  link_->AddRelativeTorque(moments);

  if (!rudder_ || ailerons_.size() == 0 || elevators_.size() == 0)
    return;

  for (int i = 0; i < ailerons_.size(); i++) {
    double aileron_angle = ailerons_.at(i)->GetAngle(0).Radian();
    ailerons_.at(i)->SetVelocity(0, 2.0 * (aileron_left_info_.deflection - aileron_angle));
  }

  for (int i = 0; i < elevators_.size(); i++) {
    double elevator_angle = elevators_.at(i)->GetAngle(0).Radian();
    elevators_.at(i)->SetVelocity(0, 2.0 * (elevator_info_.deflection - elevator_angle));
  }

  for (int i = 0; i < flaps_.size(); i++) {
    double flap_angle = flaps_.at(i)->GetAngle(0).Radian();
    flaps_.at(i)->SetVelocity(0, 2.0 * (flap_info_.deflection - flap_angle));
  }

  double rudder_angle = rudder_->GetAngle(0).Radian();
  rudder_->SetVelocity(0, 2.0 * (rudder_info_.deflection - rudder_angle));
}

void GazeboAerodynamicsPlugin::ComputeAerodynamicForcesMoments(math::Vector3& forces, math::Vector3& moments) {
  math::Quaternion orientation = link_->GetWorldPose().rot;
  math::Vector3 air_speed_body = orientation.RotateVectorReverse(link_->GetWorldLinearVel() - wind_speed_);
  //air_speed_ = link_->GetWorldLinearVel();
  //math::Vector3 lin_vel_body = orientation.RotateVectorReverse(air_speed_);
  math::Vector3 ang_vel_body = link_->GetRelativeAngularVel();

  double u = air_speed_body.x; //No wind: double u = lin_vel_body.x;
  double v = -air_speed_body.y; //No wind: double v = -lin_vel_body.y;
  double w = -air_speed_body.z; //No wind: double w = -lin_vel_body.z;

  double p = ang_vel_body.x;
  double q = -ang_vel_body.y;
  double r = -ang_vel_body.z;

  double V = air_speed_body.GetLength(); //No wind: double V = lin_vel_body.GetLength();
  double beta = (V < 0.1) ? 0.0 : asin(v / V);
  double alpha = (V < 0.1) ? 0.0 : atan(w / u);

  if (alpha > 0.27)
    alpha = 0.27;
  else if (alpha < -0.27)
    alpha = -0.27;

  double q_bar_S = 0.5 * kRhoAir * V * V * wing_surface_;

  double delta_ail_right = aileron_right_info_.deflection;
  double delta_ail_left = aileron_left_info_.deflection;
  double delta_elv = elevator_info_.deflection;
  double delta_flap = flap_info_.deflection;
  double delta_rdr = rudder_info_.deflection;

  double delta_ail_sum = delta_ail_right + delta_ail_left;
  double delta_ail_diff = delta_ail_left;// - delta_ail_right;
  double delta_flp_sum = 0.0; //2.0 * delta_flap;
  double delta_flp_diff = 0.0;

  double epsilon_thrust = 0.0;
  double cx_thrust = 0.1149;
  double cz_thrust = 0.1149;
  double x_thrust = 0.235;
  double y_thrust = 0.0;
  double z_thrust = 0.035;

  double D = q_bar_S * (aero_params_.c_D_alpha.dot(Eigen::Vector3d(1.0, alpha, alpha * alpha)) +
                        aero_params_.c_D_beta.dot(Eigen::Vector3d(0.0, beta, beta * beta)) +
                        aero_params_.c_D_delta_ail.dot(Eigen::Vector3d(0.0, delta_ail_sum, delta_ail_sum * delta_ail_sum)) +
                        aero_params_.c_D_delta_flp.dot(Eigen::Vector3d(0.0, delta_flp_sum, delta_flp_sum * delta_flp_sum)));
  double Y = q_bar_S * (aero_params_.c_Y_beta.dot(Eigen::Vector2d(0.0, beta)));
  double L = q_bar_S * (aero_params_.c_L_alpha.dot(Eigen::Vector4d(1.0, alpha, alpha * alpha, alpha * alpha * alpha)) +
                        aero_params_.c_L_delta_ail.dot(Eigen::Vector2d(0.0, delta_ail_sum)) +
                        aero_params_.c_L_delta_flp.dot(Eigen::Vector2d(0.0, delta_flp_sum)));

  double p_hat = (V < 0.1) ? 0.0 : p * wingspan_ / (2.0 * V);
  double q_hat = (V < 0.1) ? 0.0 : q * chord_length_ / (2.0 * V);
  double r_hat = (V < 0.1) ? 0.0 : r * wingspan_ / (2.0 * V);

  double Lm = q_bar_S * wingspan_ * (aero_params_.c_Lm_beta.dot(Eigen::Vector2d(0.0, beta)) +
                                     aero_params_.c_Lm_p.dot(Eigen::Vector2d(0.0, p_hat)) +
                                     aero_params_.c_Lm_r.dot(Eigen::Vector2d(0.0, r_hat)) +
                                     aero_params_.c_Lm_delta_ail.dot(Eigen::Vector2d(0.0, delta_ail_diff)) +
                                     aero_params_.c_Lm_delta_flp.dot(Eigen::Vector2d(0.0, delta_flp_diff)));
  double Mm = q_bar_S * chord_length_ * (aero_params_.c_Mm_alpha.dot(Eigen::Vector2d(1.0, alpha)) +
                                         aero_params_.c_Mm_q.dot(Eigen::Vector2d(0.0, q_hat)) +
                                         aero_params_.c_Mm_delta_elv.dot(Eigen::Vector2d(0.0, delta_elv)));
  double Nm = q_bar_S * wingspan_ * (aero_params_.c_Nm_beta.dot(Eigen::Vector2d(0.0, beta)) +
                                     aero_params_.c_Nm_r.dot(Eigen::Vector2d(0.0, r_hat)) +
                                     aero_params_.c_Nm_delta_rud.dot(Eigen::Vector2d(0.0, delta_rdr)));

  double T = aero_params_.c_T.dot(Eigen::Vector3d(1.0, throttle_, throttle_ * throttle_));
  double Tx = cos(epsilon_thrust) * T;
  double Tz = sin(epsilon_thrust) * T;

  /*double TL_d = -y_thrust * sin(epsilon_thrust) * -T;
  double TM_d = -x_thrust * sin(epsilon_thrust) * T + z_thrust * cos(epsilon_thrust) * T;
  double TN_d = y_thrust * cos(epsilon_thrust) * -T;

  double TL_ind = Tx * cx_thrust;
  double TN_ind = Tz * cz_thrust;*/

  Eigen::Vector3d force_w(-D, Y, -L);
  Eigen::Vector3d momentum_w(Lm, Mm, Nm);

  double ca = cos(alpha);
  double sa = sin(alpha);
  double cb = cos(beta);
  double sb = sin(beta);

  Eigen::Matrix3d R_WB;
  R_WB << ca * cb, sb, sa * cb,
          -sb * ca, cb, -sa * sb,
          -sa, 0.0, ca;

  Eigen::Vector3d force_T_b(Tx, 0.0, Tz);
  //Eigen::Vector3d momentum_T_b(TL_d + TL_ind, TM_d, TN_d + TN_ind);

  Eigen::Matrix3d R_WB_t = R_WB.transpose();

  Eigen::Vector3d force_b = R_WB_t * force_w + force_T_b;
  Eigen::Vector3d momentum_b = momentum_w; //R_WB_t * momentum_w; // + momentum_T_b;

  forces = math::Vector3(force_b[0], -force_b[1], -force_b[2]);
  moments = math::Vector3(momentum_b[0], -momentum_b[1], -momentum_b[2]);
}

void GazeboAerodynamicsPlugin::WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed_msg) {
  wind_speed_ = math::Vector3(wind_speed_msg->velocity.x,
                              wind_speed_msg->velocity.y,
                              wind_speed_msg->velocity.z);
}

void GazeboAerodynamicsPlugin::AirSpeedCallback(const geometry_msgs::TwistStampedConstPtr air_speed_msg) {
  air_speed_.x = air_speed_msg->twist.linear.x;
  air_speed_.y = air_speed_msg->twist.linear.y;
  air_speed_.z = air_speed_msg->twist.linear.z;
}

void GazeboAerodynamicsPlugin::CommandCallback(const mav_msgs::ActuatorsConstPtr& command_msg) {
  // Process the right aileron command
  aileron_right_info_.deflection = (aileron_right_info_.d_max + aileron_right_info_.d_min) * 0.5 +
          (aileron_right_info_.d_max - aileron_right_info_.d_min) * 0.5 *
          command_msg->normalized.at(0);

  // Process the left aileron command
  aileron_left_info_.deflection = (aileron_left_info_.d_max + aileron_left_info_.d_min) * 0.5 +
          (aileron_left_info_.d_max - aileron_left_info_.d_min) * 0.5 *
          command_msg->normalized.at(4);

  //aileron_right_info_.deflection *= 0.5;

  // Process the elevator command
  elevator_info_.deflection = (elevator_info_.d_max + elevator_info_.d_min) * 0.5 +
          (elevator_info_.d_max - elevator_info_.d_min) * 0.5 *
          command_msg->normalized.at(1);

  // Process the flap command
  flap_info_.deflection = (flap_info_.d_max + flap_info_.d_min) * 0.5 +
          (flap_info_.d_max - flap_info_.d_min) * 0.5 *
          command_msg->normalized.at(3);

  // Process the rudder command
  rudder_info_.deflection = (rudder_info_.d_max + rudder_info_.d_min) * 0.5 +
          (rudder_info_.d_max - rudder_info_.d_min) * 0.5 *
          command_msg->normalized.at(2);

  // Process the throttle command
  throttle_ = command_msg->normalized.at(5);
}

void GazeboAerodynamicsPlugin::RollPitchYawrateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference_msg) {
  double roll = roll_pitch_yawrate_thrust_reference_msg->roll;
  double pitch = roll_pitch_yawrate_thrust_reference_msg->pitch;
  double yaw_rate = roll_pitch_yawrate_thrust_reference_msg->yaw_rate;

  // Process the right aileron command
  aileron_right_info_.deflection = (aileron_right_info_.d_max + aileron_right_info_.d_min) * 0.5 +
          (aileron_right_info_.d_max - aileron_right_info_.d_min) * 0.5 * roll;

  // Process the left aileron command
  aileron_left_info_.deflection = (aileron_left_info_.d_max + aileron_left_info_.d_min) * 0.5 +
          (aileron_left_info_.d_max - aileron_left_info_.d_min) * 0.5 * roll;

  // Process the elevator command
  elevator_info_.deflection = (elevator_info_.d_max + elevator_info_.d_min) * 0.5 +
          (elevator_info_.d_max - elevator_info_.d_min) * 0.5 * pitch;

  // Process the flap command
  flap_info_.deflection = 0.0;

  // Process the rudder command
  rudder_info_.deflection = (rudder_info_.d_max + rudder_info_.d_min) * 0.5 +
          (rudder_info_.d_max - rudder_info_.d_min) * 0.5 * yaw_rate;

  // Process the throttle command
  throttle_ = roll_pitch_yawrate_thrust_reference_msg->thrust.x;
}

bool GazeboAerodynamicsPlugin::RegisterControlSurfaceCallback(
        rotors_comm::RegisterControlSurface::Request& req,
        rotors_comm::RegisterControlSurface::Response& res) {
  ControlSurface surface;

  surface.d_min = req.angle_min;
  surface.d_max = req.angle_max;
  surface.deflection = 0.0;

  if (req.surface_type == "aileron") {
    ailerons_.push_back(model_->GetJoint(req.joint_name));

    if (req.surface_side == "right")
      aileron_right_info_ = surface;
    else if (req.surface_side == "left")
      aileron_left_info_ = surface;
    else
      return false;
  }
  else if (req.surface_type == "elevator") {
    elevators_.push_back(model_->GetJoint(req.joint_name));
    elevator_info_ = surface;
  }
  else if (req.surface_type == "flap") {
    flaps_.push_back(model_->GetJoint(req.joint_name));
    flap_info_ = surface;
  }
  else if (req.surface_type == "rudder") {
    rudder_ = model_->GetJoint(req.joint_name);
    rudder_info_ = surface;
  }
  else {
    res.success = false;
    return false;
  }

  res.success = true;
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboAerodynamicsPlugin);
}
