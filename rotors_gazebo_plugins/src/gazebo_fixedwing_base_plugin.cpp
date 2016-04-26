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
  orientation_ = link_->GetWorldPose().rot;

  // Rotate the velocity from world frame into local frame
  math::Vector3 global_vel = link_->GetWorldLinearVel();
  math::Vector3 body_vel = orientation_.RotateVector(global_vel);

  // Compute the forces and moments acting on the airplane
  math::Vector3 forces;
  math::Vector3 moments;
  ComputeAerodynamicForcesAndMoments(body_vel, forces, moments);

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

void GazeboFixedWingBasePlugin::ComputeAerodynamicForcesAndMoments(math::Vector3& vel, math::Vector3 &forces, math::Vector3 &moments) {
  // Compute angle of attack
  double alpha = -atan2(vel.z, vel.x);

  // Get the elevator deflection
  //physics::JointPtr elevator = elevators_.at(0);
  //double elev_def = elevator->GetAngle(0).Radian();
  double elev_def = 0.0;

  double alpha_0 = 0.02;
  double elev_def_0 = 0.01;

  // Get the coefficients of lift and drag
  double c_L = 0.0;
  double c_L_tail = 0.0;
  double c_D = 0.0;

  double d_c_L_d_alpha = M_PI;
  double d_c_L_tail_d_elev_def = -0.5 * M_PI;
  if (alpha < alpha_stall_) {
    c_L = (alpha_0 + alpha) * d_c_L_d_alpha;
    c_L_tail = (elev_def_0 + elev_def) * d_c_L_tail_d_elev_def;
    c_D = alpha * 0.1;
  } else {
    c_L = 2.0;
    c_L_tail = 2.0;
    c_D = 0.5;
  }

  // Compute the magnitudes of the lift and drag forces
  double lift_wings = fabs(0.5 * air_density_ * c_L * total_wing_area_ * (vel.z * vel.z + vel.x * vel.x));
  double lift_tails = fabs(0.5 * air_density_ * c_L_tail * total_tail_area_ * (vel.z * vel.z + vel.x * vel.x));

  double drag = 0.5 * air_density_ * c_D * total_wing_area_ * (vel.z * vel.z + vel.x * vel.x);

  std::cout << "Wings lift: " << lift_wings << std::endl;
  std::cout << "Tails lift: " << lift_tails << std::endl;

  link_->AddForceAtRelativePosition(math::Vector3(0.0, 0.0, 0.0), math::Vector3(0.5, 0.0, 0.0));
  link_->AddForceAtRelativePosition(math::Vector3(0.0, 0.0, 0.0), math::Vector3(-1.5, 0.0, 0.0));

  // Rotate the lift and drag forces into the body frame
  //forces = math::Vector3(orientation_.RotateVector(math::Vector3(-drag, 0.0, lift_wings - lift_tails)));
  forces = math::Vector3(-drag, 0.0, lift_wings - lift_tails);

  // Compute the magnitudes of the moments
  double pitching_moment = lift_wings * 0.25 - lift_tails * 1.5;

  // Combine all the moments into one vector
  moments = math::Vector3(0.0, pitching_moment, 0.0);
}

/*math::Vector3 GazeboFixedWingBasePlugin::ComputeAerodynamicMoments(math::Vector3& vel) {
  math::Vector3 moments(0.0, 0.0, 0.0);

  // Computing the pitching moment
  for (int i = 0; i < elevators_.size(); i++) {

  }

  return math::Vector3(0.0, 0.0, 0.0);
}*/

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
