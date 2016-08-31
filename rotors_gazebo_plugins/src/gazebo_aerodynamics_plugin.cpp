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

  //std::string wind_speed_sub_topic;
  std::string air_speed_sub_topic;
  std::string command_sub_topic;
  std::string reset_model_service_name;
  //getSdfParam<std::string>(_sdf, "windSpeedSubTopic", wind_speed_sub_topic, kDefaultWindSpeedSubTopic);
  getSdfParam<std::string>(_sdf, "airSpeedSubTopic", air_speed_sub_topic, kDefaultAirSpeedSubTopic);
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic, kDefaultCommandSubTopic);
  getSdfParam<std::string>(_sdf, "resetModelServiceName", reset_model_service_name,
                           kDefaultResetModelServiceName);

  start_pose_ = model_->GetWorldPose();

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

  // Read the control surface parameters from rosparam
  //std::string surface_string = "aileron";
  //pnh.param<double>(surface_string + "/min", aileron_.d_min, kDefaultControlSurfaceDeflectionMin);
  //pnh.param<double>(surface_string + "/max", aileron_.d_max, kDefaultControlSurfaceDeflectionMax);
  pnh.param<int>("aileron/channel", aileron_channel_, kDefaultAileronChannel);

  //surface_string = "elevator";
  //pnh.param<double>(surface_string + "/min", elevator_.d_min, kDefaultControlSurfaceDeflectionMin);
  //pnh.param<double>(surface_string + "/max", elevator_.d_max, kDefaultControlSurfaceDeflectionMax);
  pnh.param<int>("elevator/channel", elevator_channel_, kDefaultElevatorChannel);

  //surface_string = "rudder";
  //pnh.param<double>(surface_string + "/min", rudder_.d_min, kDefaultControlSurfaceDeflectionMin);
  //pnh.param<double>(surface_string + "/max", rudder_.d_max, kDefaultControlSurfaceDeflectionMax);
  pnh.param<int>("rudder/channel", rudder_channel_, kDefaultRudderChannel);

  // Read the aerodynamic parameters from rosparam
  pnh.param<double>("cD0", aero_params_.cD0, kDefaultCD0);
  pnh.param<double>("cDa", aero_params_.cDa, kDefaultCDa);
  pnh.param<double>("cDa2", aero_params_.cDa2, kDefaultCDa2);
  pnh.param<double>("cL0", aero_params_.cL0, kDefaultCL0);
  pnh.param<double>("cLa", aero_params_.cLa, kDefaultCLa);
  pnh.param<double>("cLa2", aero_params_.cLa2, kDefaultCLa2);
  pnh.param<double>("cLa3", aero_params_.cLa3, kDefaultCLa3);
  pnh.param<double>("cLq", aero_params_.cLq, kDefaultCLq);
  pnh.param<double>("cLde", aero_params_.cLde, kDefaultCLde);
  pnh.param<double>("cm0", aero_params_.cm0, kDefaultCm0);
  pnh.param<double>("cma", aero_params_.cma, kDefaultCma);
  pnh.param<double>("cmq", aero_params_.cmq, kDefaultCmq);
  pnh.param<double>("cmde", aero_params_.cmde, kDefaultCmde);
  pnh.param<double>("cT0", aero_params_.cT0, kDefaultCT0);
  pnh.param<double>("cT1", aero_params_.cT1, kDefaultCT1);
  pnh.param<double>("cT2", aero_params_.cT2, kDefaultCT2);
  pnh.param<double>("tauT", aero_params_.tauT, kDefaultTauT);
  pnh.param<double>("clb", aero_params_.clb, kDefaultClb);
  pnh.param<double>("clp", aero_params_.clp, kDefaultClp);
  pnh.param<double>("clr", aero_params_.clr, kDefaultClr);
  pnh.param<double>("clda", aero_params_.clda, kDefaultClda);
  pnh.param<double>("cYb", aero_params_.cYb, kDefaultCYb);
  pnh.param<double>("cnb", aero_params_.cnb, kDefaultCnb);
  pnh.param<double>("cnp", aero_params_.cnp, kDefaultCnp);
  pnh.param<double>("cnr", aero_params_.cnr, kDefaultCnr);
  pnh.param<double>("cndr", aero_params_.cndr, kDefaultCndr);

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboAerodynamicsPlugin::OnUpdate, this, _1));

  //wind_speed_sub_ = node_handle_->subscribe(wind_speed_sub_topic, 1, &GazeboAerodynamicsPlugin::WindSpeedCallback, this);
  air_speed_sub_ = node_handle_->subscribe(air_speed_sub_topic, 1,&GazeboAerodynamicsPlugin::AirSpeedCallback, this);
  command_sub_ = node_handle_->subscribe(command_sub_topic, 1, &GazeboAerodynamicsPlugin::CommandCallback, this);

  register_control_surface_service_ =
          node_handle_->advertiseService("register_control_surface",
                                         &GazeboAerodynamicsPlugin::RegisterControlSurfaceCallback,
                                         this);

  reset_model_service_ =
          node_handle_->advertiseService(reset_model_service_name,
                                         &GazeboAerodynamicsPlugin::ResetModelCallback,
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
    ailerons_.at(i)->SetVelocity(0, 2.0 * (aileron_info_.deflection - aileron_angle));
  }

  for (int i = 0; i < elevators_.size(); i++) {
    double elevator_angle = elevators_.at(i)->GetAngle(0).Radian();
    elevators_.at(i)->SetVelocity(0, 2.0 * (elevator_info_.deflection - elevator_angle));
  }

  double rudder_angle = rudder_->GetAngle(0).Radian();
  rudder_->SetVelocity(0, 2.0 * (rudder_info_.deflection - rudder_angle));
}

void GazeboAerodynamicsPlugin::ComputeAerodynamicForcesMoments(math::Vector3& forces, math::Vector3& moments) {
  math::Quaternion orientation = link_->GetWorldPose().rot;
  //math::Vector3 air_speed_body = orientation.RotateVectorReverse(link_->GetWorldLinearVel() - wind_speed_);
  math::Vector3 lin_vel_body = orientation.RotateVectorReverse(air_speed_);
  math::Vector3 ang_vel_body = link_->GetRelativeAngularVel();

  //double u = air_speed_body.x;
  //double v = -air_speed_body.y;
  //double w = -air_speed_body.z;

  double u = lin_vel_body.x;
  double v = -lin_vel_body.y;
  double w = -lin_vel_body.z;

  double p = ang_vel_body.x;
  double q = -ang_vel_body.y;
  double r = -ang_vel_body.z;

  //double V = air_speed_body.GetLength();
  double V = lin_vel_body.GetLength();

  double beta = (V < 0.1) ? 0.0 : asin(v / V);
  double alpha = (V < 0.1) ? 0.0 : atan(w / u);

  double q_bar_S = 0.5 * kRhoAir * V * V * wing_surface_;

  double uE = elevator_info_.deflection;
  double uA = aileron_info_.deflection;
  double uR = rudder_info_.deflection;
  double uT = throttle_;

  double delta_ail_sum = 2.0 * fabs(uA);
  double delta_ail_diff = 0.0;
  double delta_flp_sum = 0.0;
  double delta_flp_diff = 0.0;

  double epsilon_thrust = 0.0;
  double x_thrust = 0.235;
  double y_thrust = 0.0;
  double z_thrust = 0.035;

  Eigen::Vector3d c_D_alpha(1.6276, 0.0903, 0.0195);
  Eigen::Vector3d c_D_beta(-0.3842, 0.0, 0.0195);
  Eigen::Vector3d c_D_delta_ail(7.5037e-6, 1.4205e-4, 0.0195);
  Eigen::Vector3d c_D_delta_flp(1.23e-5, 2.7395e-4, 0.0195);

  Eigen::Vector2d c_Y_beta(0.3846, 0.0);

  Eigen::Vector4d c_L_alpha(-50.8494, -3.4594, 5.8661, 0.3304);
  Eigen::Vector2d c_L_delta_ail(0.0048, 0.3304);
  Eigen::Vector2d c_L_delta_flp(0.0073, 0.3304);

  double D = q_bar_S * (c_D_alpha.dot(Eigen::Vector3d(alpha * alpha, alpha, 1.0)) +
                        c_D_beta.dot(Eigen::Vector3d(beta * beta, beta, 0.0)) +
                        c_D_delta_ail.dot(Eigen::Vector3d(delta_ail_sum * delta_ail_sum, delta_ail_sum, 0.0)) +
                        c_D_delta_flp.dot(Eigen::Vector3d(delta_flp_sum * delta_flp_sum, delta_flp_sum, 0.0)));
  double Y = q_bar_S * (c_Y_beta.dot(Eigen::Vector2d(beta, 0.0)));
  double L = q_bar_S * (c_L_alpha.dot(Eigen::Vector4d(alpha * alpha * alpha, alpha * alpha, alpha, 1.0)) +
                        c_L_delta_ail.dot(Eigen::Vector2d(delta_ail_sum, 0.0)) +
                        c_L_delta_flp.dot(Eigen::Vector2d(delta_flp_sum, 0.0)));

  Eigen::Vector2d c_Lm_beta(-0.0069, 0.0);
  Eigen::Vector2d c_Lm_p(-0.1024, 0.0);
  Eigen::Vector2d c_Lm_r(0.0175, 0.0);
  Eigen::Vector2d c_Lm_delta_ail(0.0016, 0.0);
  Eigen::Vector2d c_Lm_delta_flp(0.001, 0.0);

  Eigen::Vector2d c_Mm_alpha(-2.9393, -0.1173);
  Eigen::Vector2d c_Mm_q(-0.317, -0.1173);
  Eigen::Vector2d c_Mm_delta_elv(-0.0167, -0.1173);

  Eigen::Vector2d c_Nm_beta(-0.0719, 0.0);
  Eigen::Vector2d c_Nm_r(-0.0058, 0.0);
  Eigen::Vector2d c_Nm_delta_rud(8.4016e-4, 0.0);

  double Lm = q_bar_S * wingspan_ * (c_Lm_beta.dot(Eigen::Vector2d(beta, 0.0)) +
                                     c_Lm_p.dot(Eigen::Vector2d(p, 0.0)) +
                                     c_Lm_r.dot(Eigen::Vector2d(r, 0.0)) +
                                     c_Lm_delta_ail.dot(Eigen::Vector2d(delta_ail_diff, 0.0)) +
                                     c_Lm_delta_flp.dot(Eigen::Vector2d(delta_flp_diff, 0.0)));
  double Mm = q_bar_S * chord_length_ * (c_Mm_alpha.dot(Eigen::Vector2d(alpha, 1.0)) +
                                         c_Mm_q.dot(Eigen::Vector2d(q, 0.0)) +
                                         c_Mm_delta_elv.dot(Eigen::Vector2d(uE, 0.0)));
  double Nm = q_bar_S * wingspan_ * (c_Nm_beta.dot(Eigen::Vector2d(beta, 0.0)) +
                                     c_Nm_r.dot(Eigen::Vector2d(r, 0.0)) +
                                     c_Nm_delta_rud.dot(Eigen::Vector2d(uR, 0.0)));

  double T = aero_params_.cT0 + aero_params_.cT1 * uT + aero_params_.cT2 * uT * uT;
  double Tx = cos(epsilon_thrust) * T;
  double Tz = sin(epsilon_thrust) * T;

  double TL_d = -y_thrust * sin(epsilon_thrust) * -T;
  double TM_d = -x_thrust * sin(epsilon_thrust) * T + z_thrust * cos(epsilon_thrust) * T;
  double TN_d = y_thrust * cos(epsilon_thrust) * -T;

  double cx_thrust = 0.1149;
  double cz_thrust = 0.1149;

  double TL_ind = Tx * cx_thrust;
  double TN_ind = Tz * cz_thrust;

  Eigen::Vector3d force_w(-D, Y, L);
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
  Eigen::Vector3d momentum_T_b(TL_d + TL_ind, TM_d, TN_d + TN_ind);

  Eigen::Vector3d force_b = R_WB.transpose() * force_w + force_T_b;
  Eigen::Vector3d momentum_b = R_WB.transpose() * momentum_w + momentum_T_b;

  forces = math::Vector3(force_b[0], -force_b[1], -force_b[2]);
  moments = math::Vector3(momentum_b[0], -momentum_b[1], -momentum_b[2]);
}

/*void GazeboFixedWingBasePlugin::WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed_msg) {
  wind_speed_ = math::Vector3(wind_speed_msg->velocity.x,
                              wind_speed_msg->velocity.y,
                              wind_speed_msg->velocity.z);
}*/

void GazeboAerodynamicsPlugin::AirSpeedCallback(const geometry_msgs::TwistStampedConstPtr air_speed_msg) {
  air_speed_.x = air_speed_msg->twist.linear.x;
  air_speed_.y = air_speed_msg->twist.linear.y;
  air_speed_.z = air_speed_msg->twist.linear.z;
}

void GazeboAerodynamicsPlugin::CommandCallback(const mav_msgs::ActuatorsConstPtr& command_msg) {
  // Process the aileron command
  aileron_info_.deflection = (aileron_info_.d_max + aileron_info_.d_min) * 0.5 +
          (aileron_info_.d_max - aileron_info_.d_min) * 0.5 *
          command_msg->normalized.at(aileron_channel_);

  // Process the elevator command
  elevator_info_.deflection = (elevator_info_.d_max + elevator_info_.d_min) * 0.5 +
          (elevator_info_.d_max - elevator_info_.d_min) * 0.5 *
          command_msg->normalized.at(elevator_channel_);

  // Process the rudder command
  rudder_info_.deflection = (rudder_info_.d_max + rudder_info_.d_min) * 0.5 +
          (rudder_info_.d_max - rudder_info_.d_min) * 0.5 *
          command_msg->normalized.at(rudder_channel_);

  // Process the throttle command
  throttle_ = command_msg->normalized.at(3);
}

bool GazeboAerodynamicsPlugin::RegisterControlSurfaceCallback(
        rotors_comm::RegisterControlSurface::Request& req,
        rotors_comm::RegisterControlSurface::Response& res) {
  ControlSurface surface;

  surface.d_min = req.angle_min;
  surface.d_max = req.angle_max;
  surface.deflection = 0.0;

  if (req.surface_type == "aileron") {
    aileron_info_ = surface;
    ailerons_.push_back(model_->GetJoint(req.joint_name));
  }
  else if (req.surface_type == "elevator") {
    elevator_info_ = surface;
    elevators_.push_back(model_->GetJoint(req.joint_name));
  }
  else if (req.surface_type == "rudder") {
    rudder_info_ = surface;
    rudder_ = model_->GetJoint(req.joint_name);
  }
  else {
    res.success = false;
    return false;
  }

  res.success = true;
  return true;
}

bool GazeboAerodynamicsPlugin::ResetModelCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  model_->SetWorldPose(start_pose_);

  math::Vector3 lin_vel = model_->GetWorldLinearVel();
  lin_vel.x = 0.0;
  lin_vel.y = 0.0;
  lin_vel.z = 0.0;
  model_->SetLinearVel(lin_vel);

  math::Vector3 ang_vel = model_->GetWorldAngularVel();
  ang_vel.x = 0.0;
  ang_vel.y = 0.0;
  ang_vel.z = 0.0;
  model_->SetAngularVel(ang_vel);

  aileron_info_.deflection = 0.0;
  for (int i = 0; i < ailerons_.size(); i++) {
    ailerons_.at(i)->SetPosition(0, 0.0);
  }

  elevator_info_.deflection = 0.0;
  for (int i = 0; i < elevators_.size(); i++) {
    elevators_.at(i)->SetPosition(0, 0.0);
  }

  rudder_info_.deflection = 0.0;
  rudder_->SetPosition(0, 0.0);

  throttle_ = 0.0;

  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboAerodynamicsPlugin);
}
