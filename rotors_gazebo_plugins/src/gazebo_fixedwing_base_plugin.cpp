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
      throttle_(0.0) {
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

  std::string air_speed_sub_topic;
  std::string command_sub_topic;
  std::string reset_topic;
  getSdfParam<std::string>(_sdf, "airSpeedSubTopic", air_speed_sub_topic, kDefaultAirSpeedSubTopic);
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic, kDefaultCommandSubTopic);
  getSdfParam<std::string>(_sdf, "resetTopic", reset_topic, kDefaultResetSubTopic);

  cam_parent_frame_ = "world";
  cam_child_frame_ = "techpod/camera_downward_link";

  /*link_->SetGravityMode(false);
  link_->SetKinematic(true);
  for (int i = 0; i < link_->GetChildCount(); i++) {
    physics::LinkPtr child = boost::dynamic_pointer_cast<gazebo::physics::Link>(link_->GetChild(i));
    if (child) {
      child->SetGravityMode(false);
      child->SetKinematic(true);
    }
  }*/

  start_position_ = model_->GetWorldPose().pos;

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
  std::string surface_string = "aileron";
  pnh.param<double>(surface_string + "/min", aileron_.d_min, kDefaultControlSurfaceDeflectionMin);
  pnh.param<double>(surface_string + "/max", aileron_.d_max, kDefaultControlSurfaceDeflectionMax);
  pnh.param<int>(surface_string + "/channel", aileron_.channel, kDefaultAileronChannel);

  surface_string = "elevator";
  pnh.param<double>(surface_string + "/min", elevator_.d_min, kDefaultControlSurfaceDeflectionMin);
  pnh.param<double>(surface_string + "/max", elevator_.d_max, kDefaultControlSurfaceDeflectionMax);
  pnh.param<int>(surface_string + "/channel", elevator_.channel, kDefaultElevatorChannel);

  surface_string = "rudder";
  pnh.param<double>(surface_string + "/min", rudder_.d_min, kDefaultControlSurfaceDeflectionMin);
  pnh.param<double>(surface_string + "/max", rudder_.d_max, kDefaultControlSurfaceDeflectionMax);
  pnh.param<int>(surface_string + "/channel", rudder_.channel, kDefaultRudderChannel);

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
          boost::bind(&GazeboFixedWingBasePlugin::OnUpdate, this, _1));

  air_speed_sub_ = node_handle_->subscribe(air_speed_sub_topic, 1, &GazeboFixedWingBasePlugin::AirSpeedCallback, this);
  command_sub_ = node_handle_->subscribe(command_sub_topic, 1, &GazeboFixedWingBasePlugin::CommandCallback, this);
  reset_sub_ = node_handle_->subscribe(reset_topic, 1, &GazeboFixedWingBasePlugin::ResetCallback, this);
}

void GazeboFixedWingBasePlugin::OnUpdate(const common::UpdateInfo& _info) {
  math::Vector3 forces;
  math::Vector3 moments;

  ComputeAerodynamicForcesMoments(forces, moments);

  link_->AddLinkForce(forces);
  link_->AddRelativeTorque(moments);

  // Broadcast the transform to the camera link
  math::Pose cam_pose = link_->GetWorldPose();
  math::Quaternion base_ori = link_->GetWorldPose().rot;
  math::Quaternion shift = math::Quaternion(0.0, M_PI, M_PI);
  math::Quaternion cam_ori = shift * base_ori;
  tf::Quaternion rot(cam_ori.x, cam_ori.y, cam_ori.z, cam_ori.w);
  tf::Vector3 pos(cam_pose.pos.x, cam_pose.pos.y, cam_pose.pos.z);
  tf_ = tf::Transform(rot, pos);
  transform_broadcaster_.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), cam_parent_frame_, cam_child_frame_));

  // Broadcast the transform to the base link
  /*tf::Quaternion base_rot(base_ori.x, base_ori.y, base_ori.z, base_ori.w);
  tf::Transform base_tf_(base_rot, pos);
  transform_broadcaster_.sendTransform(tf::StampedTransform(base_tf_, ros::Time::now(), cam_parent_frame_, "base_link"));

  math::Vector3 linear_vel_B = math::Vector3(10.0, 0.0, 0.0);
  math::Vector3 linear_vel_W = base_ori.RotateVector(linear_vel_B);
  link_->SetLinearVel(linear_vel_W);*/
}

void GazeboFixedWingBasePlugin::ComputeAerodynamicForcesMoments(math::Vector3& forces, math::Vector3& moments) {
  // Rotate airspeed into the body frame and get the body angular rates from Gazebo
  math::Quaternion orientation = link_->GetWorldPose().rot;
  math::Vector3 lin_vel_body = orientation.RotateVectorReverse(air_speed_);
  math::Vector3 ang_vel_body = link_->GetRelativeAngularVel();

  double u = lin_vel_body.x;
  double v = lin_vel_body.y;
  double w = lin_vel_body.z;

  double p = ang_vel_body.x;
  double q = ang_vel_body.y;
  double r = ang_vel_body.z;

  double V = lin_vel_body.GetLength();

  double beta = (V < 0.1) ? 0.0 : asin(v / V);
  double alpha = (V < 0.1) ? 0.0 : atan(w / u);

  double q_bar_S = 0.5 * kRhoAir * V * V * wing_surface_;

  double ca = cos(alpha);
  double sa = sin(alpha);

  math::Matrix3 H_W2B(ca, 0.0, -sa, 0.0, 1.0, 0.0, sa, 0.0, ca);

  // Compute the force coefficients
  double cD = aero_params_.cD0 + aero_params_.cDa * alpha + aero_params_.cDa2 * alpha * alpha;
  double cY_s = aero_params_.cYb * beta;
  double cL = aero_params_.cL0 + aero_params_.cLa * alpha + aero_params_.cLa2 * alpha * alpha + aero_params_.cLa3 * alpha * alpha * alpha;

  math::Vector3 cXYZ = H_W2B * math::Vector3(-cD, cY_s, -cL);

  // Compute the forces
  double X = q_bar_S * cXYZ.x;
  double Y = q_bar_S * cXYZ.y;
  double Z = q_bar_S * cXYZ.z;

  double p_hat = (V < 0.1) ? 0.0 : p * wingspan_ / (2.0 * V);
  double q_hat = (V < 0.1) ? 0.0 : q * chord_length_ / (2.0 * V);
  double r_hat = (V < 0.1) ? 0.0 : r * wingspan_ / (2.0 * V);

  // Get the control surfaces and throttle commands
  double uE = elevator_.deflection;
  double uA = aileron_.deflection;
  double uR = rudder_.deflection;
  double uT = throttle_;

  // Compute the moment coefficients
  double cl_s = aero_params_.clb * beta + aero_params_.clp * p_hat + aero_params_.clr * r_hat + aero_params_.clda * uA;
  double cm_s = aero_params_.cm0 + aero_params_.cma * alpha + aero_params_.cmq * q_hat + aero_params_.cmde * uE;
  double cn_s = aero_params_.cnb * beta + aero_params_.cnp * p_hat + aero_params_.cnr * r_hat + aero_params_.cndr * uR;

  // Compute the moments
  double Lm = q_bar_S * wingspan_ * cl_s;
  double Mm = q_bar_S * chord_length_ * cm_s;
  double Nm = q_bar_S * wingspan_ * cn_s;

  // Thrust force
  double T = aero_params_.cT0 + aero_params_.cT1 * uT + aero_params_.cT2 * uT * uT;

  // Update the forces and moments vectors
  forces = math::Vector3(T + X, Y, Z);
  moments = math::Vector3(Lm, Mm, Nm);
}

void GazeboFixedWingBasePlugin::AirSpeedCallback(const geometry_msgs::Vector3ConstPtr& air_speed_msg) {
  air_speed_.x = air_speed_msg->x;
  air_speed_.y = air_speed_msg->y;
  air_speed_.z = air_speed_msg->z;
}

void GazeboFixedWingBasePlugin::CommandCallback(const mav_msgs::ActuatorsConstPtr& command_msg) {
  aileron_.deflection = (aileron_.d_max + aileron_.d_min) * 0.5 + (aileron_.d_max - aileron_.d_min) * 0.5 * command_msg->normalized.at(aileron_.channel);
  elevator_.deflection = (elevator_.d_max + elevator_.d_min) * 0.5 + (elevator_.d_max - elevator_.d_min) * 0.5 * command_msg->normalized.at(elevator_.channel);
  rudder_.deflection = (rudder_.d_max + rudder_.d_min) * 0.5 + (rudder_.d_max - rudder_.d_min) * 0.5 * command_msg->normalized.at(rudder_.channel);

  throttle_ = command_msg->normalized.at(3);
}

void GazeboFixedWingBasePlugin::ResetCallback(const std_msgs::BoolConstPtr& reset_msg) {
  if (reset_msg->data) {
    math::Pose pose = model_->GetWorldPose();
    pose.pos = start_position_;
    pose.rot = math::Quaternion(0.0, 0.0, 0.0);
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

    aileron_.deflection = 0.0;
    elevator_.deflection = 0.0;
    rudder_.deflection = 0.0;
    throttle_ = 0.0;
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFixedWingBasePlugin);
}
