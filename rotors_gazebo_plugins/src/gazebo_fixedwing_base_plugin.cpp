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
      total_wing_area_(0.0),
      total_tail_area_(0.0) {
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

  std::string reset_topic;
  getSdfParam<std::string>(_sdf, "resetTopic", reset_topic, kDefaultResetSubTopic);
  getSdfParam<double>(_sdf, "airDensity", air_density_, kDefaultAirDensity);
  getSdfParam<double>(_sdf, "alphaStall", alpha_stall_, kDefaultAlphaStall);

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboFixedWingBasePlugin::OnUpdate, this, _1));

  reset_sub_ = node_handle_->subscribe(reset_topic, 1, &GazeboFixedWingBasePlugin::resetCallback, this);

  register_aero_surface_service_ =
          node_handle_->advertiseService("register_aero_surface", &GazeboFixedWingBasePlugin::RegisterAeroSurface, this);

  orientation_ = math::Quaternion(1.0, 0.0, 0.0, 0.0);
}

void GazeboFixedWingBasePlugin::OnUpdate(const common::UpdateInfo& _info) {
  //if (elevators_.size() == 0)
  //  return;

  // Get the orientation of the airplane in world frame
  //orientation_ = link_->GetWorldPose().rot;

  // Rotate the velocity from world frame into local frame
  //math::Vector3 global_vel = link_->GetWorldLinearVel();
  //math::Vector3 body_vel = orientation_.RotateVector(global_vel);

  // Compute the forces and moments acting on the airplane
  math::Vector3 forces;
  math::Vector3 moments;
  ComputeAerodynamicForcesAndMoments(forces, moments);

  //std::cout << " " << std::endl;
  //std::cout << "Forces: " << forces.x << ", " << forces.y << ", " << forces.z << std::endl;
  //std::cout << "Moments: " << moments.x << ", " << moments.y << ", " << moments.z << std::endl;

  // Apply forces and moments to the link
  //link_->AddForce(forces);
  //link_->AddForceAtRelativePosition(forces, math::Vector3(0.5, 0.0, 0.0));
  //link_->AddRelativeTorque(moments);
}

bool GazeboFixedWingBasePlugin::RegisterAeroSurface(
        rotors_gazebo_plugins::RegisterAeroSurface::Request& req,
        rotors_gazebo_plugins::RegisterAeroSurface::Response& res) {
  physics::LinkPtr surface_link;

  switch(req.surface_type) {
    case AERO_SURFACE_TYPE_WING:
      total_wing_area_ += req.surface_area;
      break;
    case AERO_SURFACE_TYPE_TAIL:
      total_tail_area_ += req.surface_area;
      break;
    case AERO_SURFACE_TYPE_AILERON:
      surface_link = model_->GetLink(req.link_name);
      ailerons_.push_back(surface_link->GetParentJoints().at(0));
      break;
    case AERO_SURFACE_TYPE_ELEVATOR:
      surface_link = model_->GetLink(req.link_name);
      elevators_.push_back(surface_link->GetParentJoints().at(0));
      break;
    case AERO_SURFACE_TYPE_RUDDER:
      surface_link = model_->GetLink(req.link_name);
      rudder_ = surface_link->GetParentJoints().at(0);
      break;
    default:
      res.success = false;
      return false;
  }

  res.success = true;
  return true;
}

void GazeboFixedWingBasePlugin::ComputeAerodynamicForcesAndMoments(math::Vector3 &forces, math::Vector3 &moments) {
  // Get the body frame linear and angular velocities
  math::Vector3 lin_vel = link_->GetRelativeLinearVel();
  math::Vector3 ang_vel = link_->GetRelativeAngularVel();

  lin_vel = lin_vel.Normalize();

  double p = ang_vel.x;
  double q = ang_vel.y;
  double r = ang_vel.z;

  double V = lin_vel.x;
  double beta = lin_vel.y;
  double alpha = lin_vel.z;

  double p_hat = p * kBWing / (2.0 * V);
  double q_hat = q * kCChord / (2.0 * V);
  double r_hat = r * kBWing / (2.0 * V);

  // Get the control surfaces deflections
  double uE = elevators_.at(0)->GetAngle(0).Radian();
  double uA = ailerons_.at(0)->GetAngle(0).Radian();
  double uR = rudder_->GetAngle(0).Radian();

  double q_bar_S = 0.5 * kRhoAir * V * V * kSWing;

  // Compute the moment coefficients
  double cl_s = kClb * beta + kClp * p_hat + kClr * r_hat + kClda * uA;
  double cm_s = kCm0 + kCma * alpha + kCmq * q_hat + kCmde * uE;
  double cn_s = kCnb * beta + kCnp * p_hat + kCnr * r_hat + kCndr * uR;

  // Compute the moments
  double Lm = q_bar_S * kBWing * cl_s;
  double Mm = q_bar_S * kCChord * cm_s;
  double Nm = q_bar_S * kBWing * cn_s;

  double ca = cos(alpha);
  double sa = sin(alpha);

  math::Matrix3 H_W2B(ca, 0.0, -sa, 0.0, 1.0, 0.0, sa, 0.0, ca);

  double cD = kCD0 + kCDa * alpha + kCDa2 * alpha * alpha;
  double cY_s = kCYb * beta;
  double cL = kCL0 + kCLa * alpha + kCLa2 * alpha * alpha + kCLa3 * alpha * alpha * alpha;

  math::Vector3 cXYZ = H_W2B * math::Vector3(-cD, cY_s, -cL);

  // Compute the forces
  double X = q_bar_S * cXYZ.x;
  double Y = q_bar_S * cXYZ.y;
  double Z = q_bar_S * cXYZ.z;

  forces = math::Vector3(X, Y, Z);
  moments = math::Vector3(Lm, Mm, Nm);
}

void GazeboFixedWingBasePlugin::resetCallback(const std_msgs::BoolConstPtr& reset_msg) {
  if (reset_msg->data) {
    math::Pose pose = model_->GetWorldPose();
    pose.pos.x = 0.0;
    pose.pos.y = 0.0;
    pose.pos.z = 0.0;
    pose.rot.w = 1.0;
    pose.rot.x = 0.0;
    pose.rot.y = 0.0;
    pose.rot.z = 0.0;
    model_->SetWorldPose(pose);

    math::Vector3 lin_vel = model_->GetWorldLinearVel();
    lin_vel.x = 0.0;
    lin_vel.y = 0.0;
    lin_vel.z = 0.0;
    model_->SetLinearVel(lin_vel);

    math::Vector3 lin_acc = model_->GetWorldLinearAccel();
    lin_acc.x = 0.0;
    lin_acc.y = 0.0;
    lin_acc.z = 0.0;
    model_->SetLinearAccel(lin_acc);

    math::Vector3 ang_vel = model_->GetWorldAngularVel();
    ang_vel.x = 0.0;
    ang_vel.y = 0.0;
    ang_vel.z = 0.0;
    model_->SetAngularVel(ang_vel);

    math::Vector3 ang_acc = model_->GetWorldAngularAccel();
    ang_acc.x = 0.0;
    ang_acc.y = 0.0;
    ang_acc.z = 0.0;
    model_->SetAngularAccel(ang_acc);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFixedWingBasePlugin);
}
