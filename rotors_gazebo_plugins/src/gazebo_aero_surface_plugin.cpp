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

#include "rotors_gazebo_plugins/gazebo_aero_surface_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboAeroSurfacePlugin::GazeboAeroSurfacePlugin()
    : ModelPlugin(),
      node_handle_(0),
      ref_angle_(0.0) {}

GazeboAeroSurfacePlugin::~GazeboAeroSurfacePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboAeroSurfacePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    node_handle_ = new ros::NodeHandle(namespace_);
  }
  else
    gzerr << "[gazebo_aero_surface_plugin] Please specify a robotNamespace.\n";

  // Get the pointer to the joint
  if (_sdf->HasElement("jointName")) {
    std::string joint_name = _sdf->GetElement("jointName")->Get<std::string>();
    joint_ = model_->GetJoint(joint_name);
    if (joint_ == NULL)
      gzthrow("[gazebo_aero_surface_plugin] Couldn't find specified joint \"" << joint_name << "\".");
  }
  else
    gzerr << "[gazebo_aero_surface_plugin] Please specify a jointName.\n";

  /*if (_sdf->HasElement("surfaceType")) {
    std::string surface_type = _sdf->GetElement("surfaceType")->Get<std::string>();
    if (surface_type == "wing")
      surface_type_ = AERO_SURFACE_TYPE_WING;
    else if (surface_type == "tail")
      surface_type_ = AERO_SURFACE_TYPE_TAIL;
    else if (surface_type == "aileron")
      surface_type_ = AERO_SURFACE_TYPE_AILERON;
    else if (surface_type == "elevator")
      surface_type_ = AERO_SURFACE_TYPE_ELEVATOR;
    else if (surface_type == "rudder")
      surface_type_ = AERO_SURFACE_TYPE_RUDDER;
    else
      gzerr << "[gazebo_aero_surface_plugin] Please only use 'wing', 'aileron', 'elevator', or 'rudder' as surfaceType.\n";
  }
  else
    gzerr << "[gazebo_aero_surface_plugin] Please specify the surface type ('wing', 'aileron', 'elevator', or 'rudder').\n";

  if (_sdf->HasElement("positiveDirection")) {
    std::string positive_direction = _sdf->GetElement("positiveDirection")->Get<std::string>();
    if (positive_direction == "cw")
      positive_direction_ = positive_direction::CW;
    else if (positive_direction == "ccw")
      positive_direction_ = positive_direction::CCW;
    else
      gzerr << "[gazebo_aero_surface_plugin] Please only use 'cw' or 'ccw' as positiveDirection.\n";
  }
  else
    gzerr << "[gazebo_aero_surface_plugin] Please specify a positive direction ('cw' or 'ccw').\n";

  std::string command_sub_topic;
  double surface_area;

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic, kDefaultCommandSubTopic);
  getSdfParam<double>(_sdf, "surfaceArea", surface_area, kDefaultSurfaceArea);
  getSdfParam<double>(_sdf, "gain", gain_, kDefaultGain);
  getSdfParam<double>(_sdf, "minAngle", min_angle_, kDefaultMinAngle);
  getSdfParam<double>(_sdf, "maxAngle", max_angle_, kDefaultMaxAngle);
  getSdfParam<int>(_sdf, "channel", channel_, kDefaultChannel);

  register_aero_surface_client_ = node_handle_->serviceClient<rotors_gazebo_plugins::RegisterAeroSurface>("register_aero_surface");

  rotors_gazebo_plugins::RegisterAeroSurface aero_surface_info;
  aero_surface_info.request.link_name = joint_->GetChild()->GetName();
  aero_surface_info.request.surface_area = surface_area;
  aero_surface_info.request.surface_type = surface_type_;

  if (!register_aero_surface_client_.call(aero_surface_info))
    gzerr << "[gazebo_aero_surface_plugin] Unable to register the aerodynamic surface info.\n";

  if (surface_type_ == AERO_SURFACE_TYPE_AILERON || surface_type_ == AERO_SURFACE_TYPE_ELEVATOR  ||
          surface_type_ == AERO_SURFACE_TYPE_RUDDER) {
    damping_ = joint_->GetDamping(0);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ =
        event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GazeboAeroSurfacePlugin::OnUpdate, this, _1));

    command_sub_ = node_handle_->subscribe(command_sub_topic, 1, &GazeboAeroSurfacePlugin::AngleCallback, this);
  }*/
}

void GazeboAeroSurfacePlugin::AngleCallback(const mav_msgs::ActuatorsConstPtr& servo_angles) {
  ROS_ASSERT_MSG(servo_angles->angles.size() > channel_,
                 "You tried to access index %d of the Angles message array which is of size %d.",
                 channel_, servo_angles->angles.size());
  ref_angle_ = positive_direction_ *
          ((max_angle_ + min_angle_) * 0.5 + (max_angle_ - min_angle_) * 0.5 * servo_angles->angles.at(channel_));
}

// This gets called by the world update start event.
void GazeboAeroSurfacePlugin::OnUpdate(const common::UpdateInfo& _info) {
  //double current_angle = joint_->GetAngle(0).Radian();
  //double err = ref_angle_ - current_angle;
  //joint_->SetVelocity(0, err * gain_);
  //joint_->SetForce(0, damping_ + err * gain_);
  //joint_->SetPosition(0, ref_angle_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboAeroSurfacePlugin);
}
