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

#include "rotors_gazebo_plugins/gazebo_fixedwing_base_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboFixedWingBasePlugin::GazeboFixedWingBasePlugin()
    : ModelPlugin(),
      node_handle_(0),
      throttle_(0.0),
      aileron_deflection_(0.0),
      elevator_deflection_(0.0),
      rudder_deflection_(0.0) {
}

GazeboFixedWingBasePlugin::~GazeboFixedWingBasePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboFixedWingBasePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model and the world
  model_ = _model;
  world_ = model_->GetWorld();

  // Get the robot namespace
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_fixedwing_base_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  // Get the link name
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_fixedwing_base_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_fixedwing_base_plugin] Couldn't find specified link \"" << link_name << "\".");

  // Disable gravity and make the base link and its child links kinematic
  link_->SetGravityMode(false);
  link_->SetKinematic(true);
  std::vector<physics::LinkPtr> child_links = link_->GetChildJointsLinks();
  for (int i = 0; i < child_links.size(); i++) {
    child_links.at(i)->SetGravityMode(false);
    child_links.at(i)->SetKinematic(true);
  }

  std::string air_speed_sub_topic;
  std::string command_sub_topic;
  std::string reset_topic;
  getSdfParam<std::string>(_sdf, "airSpeedSubTopic", air_speed_sub_topic, kDefaultAirSpeedSubTopic);
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic, kDefaultCommandSubTopic);
  getSdfParam<std::string>(_sdf, "resetTopic", reset_topic, kDefaultResetSubTopic);

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboFixedWingBasePlugin::OnUpdate, this, _1));

  //air_speed_sub_ = node_handle_->subscribe(air_speed_sub_topic, 1, &GazeboFixedWingBasePlugin::AirSpeedCallback, this);
  command_sub_ = node_handle_->subscribe(command_sub_topic, 1, &GazeboFixedWingBasePlugin::CommandCallback, this);
  reset_sub_ = node_handle_->subscribe(reset_topic, 1, &GazeboFixedWingBasePlugin::ResetCallback, this);

  last_sim_time_ = world_->GetSimTime();

  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init(world_->GetName());

  std::string topicname = "~/" + model_->GetName() + "/linear_accel";

  linear_accel_pub_ = gazebo_node_->Advertise<gazebo::msgs::Vector3d>(topicname);
}

void GazeboFixedWingBasePlugin::OnUpdate(const common::UpdateInfo& _info) {
  UpdateKinematics();
}

void GazeboFixedWingBasePlugin::UpdateKinematics() {
  // Compute the time difference since last update
  common::Time current_time = world_->GetSimTime();
  double dt = current_time.Double() - last_time_.Double();
  last_time_ = current_time;

  // Get the body frame linear and angular velocities
  math::Vector3 lin_vel_body = link_->GetRelativeLinearVel();
  math::Vector3 ang_vel_body = link_->GetRelativeAngularVel();

  // Get the current orientation
  math::Quaternion orientation = link_->GetWorldPose().rot;
  orientation = math::Quaternion(orientation.GetRoll() - M_PI, orientation.GetPitch(), orientation.GetYaw());
  //double phi = orientation.GetRoll() - M_PI;
  //double theta = -orientation.GetPitch();
  //double psi = -orientation.GetYaw();
  double phi = orientation.GetRoll();
  double theta = orientation.GetPitch();

  double p = ang_vel_body.x;
  double q = ang_vel_body.y;
  double r = ang_vel_body.z;

  double V = lin_vel_body.GetLength();

  double beta = (V < 0.1) ? 0.0 : asin(lin_vel_body.y / V);
  double alpha = (V < 0.1) ? 0.0 : atan(lin_vel_body.z / lin_vel_body.x);

  double q_bar_S = 0.5 * kRhoAir * V * V * kSWing;

  double ca = cos(alpha);
  double sa = sin(alpha);

  math::Matrix3 H_W2B(ca, 0.0, -sa, 0.0, 1.0, 0.0, sa, 0.0, ca);

  // Compute the force coefficients
  double cD = kCD0 + kCDa * alpha + kCDa2 * alpha * alpha;
  double cY_s = kCYb * beta;
  double cL = kCL0 + kCLa * alpha + kCLa2 * alpha * alpha + kCLa3 * alpha * alpha * alpha;

  math::Vector3 cXYZ = H_W2B * math::Vector3(-cD, cY_s, -cL);

  // Compute the forces
  double X = q_bar_S * cXYZ.x;
  double Y = q_bar_S * cXYZ.y;
  double Z = q_bar_S * cXYZ.z;

  double p_hat = (V < 0.1) ? 0.0 : p * kBWing / (2.0 * V);
  double q_hat = (V < 0.1) ? 0.0 : q * kCChord / (2.0 * V);
  double r_hat = (V < 0.1) ? 0.0 : r * kBWing / (2.0 * V);

  // Get the control surfaces and throttle commands
  double uE = elevator_deflection_;
  double uA = aileron_deflection_;
  double uR = rudder_deflection_;
  double uT = throttle_;

  // Compute the moment coefficients
  double cl_s = kClb * beta + kClp * p_hat + kClr * r_hat + kClda * uA;
  double cm_s = kCm0 + kCma * alpha + kCmq * q_hat + kCmde * uE;
  double cn_s = kCnb * beta + kCnp * p_hat + kCnr * r_hat + kCndr * uR;

  // Compute the moments
  double Lm = q_bar_S * kBWing * cl_s;
  double Mm = q_bar_S * kCChord * cm_s;
  double Nm = q_bar_S * kBWing * cn_s;

  // Thrust force
  double T = kCT0 + kCT1 * uT + kCT2 * uT * uT;

  // Intermediate states
  double u = V * cos(alpha) * cos(beta);
  double v = V * sin(beta);
  double w = V * sin(alpha) * cos(beta);

  // Intermediate state differentials
  double u_dot = r * v - q * w - kG * sin(theta) + (X + T * cos(kIThrust)) / kMass;
  double v_dot = p * w - r * u + kG * sin(phi) * cos(theta) + Y / kMass;
  double w_dot = q * u - p * v + kG * cos(phi) * cos(theta) + (Z + T * sin(kIThrust)) / kMass;

  double I1 = kIxz * (kIyy - kIxx - kIzz);
  double I2 = (kIxx * kIzz - kIxz * kIxz);
  double p_dot = (kIzz * Lm + kIxz * Nm - (I1 * p + (kIxz * kIxz + kIzz * (kIzz - kIyy)) * r) * q) / I2;
  double q_dot = (Mm - (kIxx - kIzz) * p * r - kIxz * (p * p - r * r)) / kIyy;
  double r_dot = (kIxz * Lm + kIxx * Nm + (I1 * r + (kIxz * kIxz + kIxx * (kIxx - kIyy)) *p) *q) / I2;

  // Remove the acceleration due to gravity
  //math::Vector3 gravity(0, 0, kG);
  math::Vector3 gravity = orientation.RotateVectorReverse(math::Vector3(0, 0, kG));

  u_dot = u_dot - gravity.x;
  v_dot = v_dot - gravity.y;
  w_dot = w_dot - gravity.z;

  // Compute the new linear velocity
  u = u + u_dot * dt;
  v = v + v_dot * dt;
  w = w + w_dot * dt;

  // Rotate the linear velocity into world frame
  math::Vector3 new_lin_vel_w = orientation.RotateVector(math::Vector3(u, v, w));

  // Compute the new angular velocity
  p = p + p_dot * dt;
  q = q + q_dot * dt;
  r = r + r_dot * dt;

  // Rotate the angular velocities into the world frame
  double phi_dot = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
  double theta_dot = q * cos(phi) - r * sin(phi);
  double psi_dot = (q * sin(phi) + r * cos(phi)) / cos(theta);

  math::Vector3 new_ang_vel_w = math::Vector3(phi_dot, theta_dot, psi_dot);

  // Set the new velocities
  link_->SetLinearVel(new_lin_vel_w);
  link_->SetAngularVel(new_ang_vel_w);

  gazebo::msgs::Vector3d accel_msg;

  #if GAZEBO_MAJOR_VERSION < 6
    gazebo::msgs::Set(&accel_msg, gazebo::math::Vector3(u_dot, v_dot, w_dot));
  #else
    gazebo::msgs::Set(&accel_msg, ignition::math::Vector3d(u_dot, v_dot, w_dot));
  #endif

  linear_accel_pub_->Publish(accel_msg);
}

void GazeboFixedWingBasePlugin::AirSpeedCallback(const geometry_msgs::Vector3ConstPtr& air_speed_msg) {
  air_speed_.x = air_speed_msg->x * -1.0;
  air_speed_.y = air_speed_msg->y * -1.0;
  air_speed_.z = air_speed_msg->z * -1.0;
}

void GazeboFixedWingBasePlugin::CommandCallback(const mav_msgs::ActuatorsConstPtr& command_msg) {
  aileron_deflection_ = (kDeflectionMax + kDeflectionMin) * 0.5 + (kDeflectionMax - kDeflectionMin) * 0.5 * command_msg->normalized.at(0);
  elevator_deflection_ = (kDeflectionMax + kDeflectionMin) * 0.5 + (kDeflectionMax - kDeflectionMin) * 0.5 * command_msg->normalized.at(1);
  rudder_deflection_ = (kDeflectionMax + kDeflectionMin) * 0.5 + (kDeflectionMax - kDeflectionMin) * 0.5 * command_msg->normalized.at(2);

  throttle_ = command_msg->normalized.at(3);
}

void GazeboFixedWingBasePlugin::ResetCallback(const std_msgs::BoolConstPtr& reset_msg) {
  if (reset_msg->data) {
    math::Pose pose = model_->GetWorldPose();
    pose.pos = math::Vector3(0.0, 0.0, 0.0);
    pose.rot = math::Quaternion(M_PI, 0.0, 0.0);
    model_->SetWorldPose(pose);

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

    aileron_deflection_ = 0.0;
    elevator_deflection_ = 0.0;
    rudder_deflection_ = 0.0;
    throttle_ = 0.0;

    gazebo::msgs::Vector3d accel_msg;

    #if GAZEBO_MAJOR_VERSION < 6
      gazebo::msgs::Set(&accel_msg, gazebo::math::Vector3(0.0, 0.0, 0.0));
    #else
      gazebo::msgs::Set(&accel_msg, ignition::math::Vector3d(0.0, 0.0, 0.0));
    #endif

    linear_accel_pub_->Publish(accel_msg);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFixedWingBasePlugin);
}
