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

#include "rotors_gazebo_plugins/gazebo_gps_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboGpsPlugin::GazeboGpsPlugin()
    : ModelPlugin(),
      node_handle_(0),
      gps_sequence_(0) {
}

GazeboGpsPlugin::~GazeboGpsPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboGpsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model and the world
  model_ = _model;
  world_ = model_->GetWorld();

  // Use the robot namespace to create the node handle
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  // Use the link name as the frame id
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_gps_plugin] Couldn't find specified link \"" << link_name << "\".");

  frame_id_ = link_name;

  // Retrieve the rest of the SDF parameters
  getSdfParam<std::string>(_sdf, "gpsTopic", gps_topic_, mav_msgs::default_topics::GPS);
  getSdfParam<double>(_sdf, "referenceLatitude", ref_lat_, kDefaultRefLat);
  getSdfParam<double>(_sdf, "referenceLongitude", ref_lon_, kDefaultRefLon);
  getSdfParam<double>(_sdf, "referenceAltitude", ref_alt_, kDefaultRefAlt);
  getSdfParam<double>(_sdf, "referenceHeading", ref_heading_, kDefaultRefHeading);

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboGpsPlugin::OnUpdate, this, _1));

  gps_pub_ = node_handle_->advertise<sensor_msgs::NavSatFix>(gps_topic_, 1);

  // Fill gps message
  gps_message_.header.frame_id = frame_id_;
  gps_message_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gps_message_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  gps_message_.latitude = ref_lat_;
  gps_message_.longitude = ref_lon_;
  gps_message_.altitude = ref_alt_;
  gps_message_.position_covariance.fill(0.0);

  double temp = 1.0 / (1.0 - kEccentrity2 * sin(ref_lat_ * M_PI / 180.0) * sin(ref_lat_ * M_PI / 180.0));
  double prime_vertical_radius = kEquatorialRadius * sqrt(temp);
  radius_north_ = prime_vertical_radius * (1 - kEccentrity2) * temp;
  radius_east_  = prime_vertical_radius * cos(ref_lat_ * M_PI / 180.0);
  ref_heading_ = ref_heading_ * M_PI / 180.0;
}

void GazeboGpsPlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();

  // Get the current pose from Gazebo
  math::Vector3 position = model_->GetWorldPose().pos;

  // Update the GPS coordinates
  gps_message_.latitude = ref_lat_ + (cos(ref_heading_) * position.x + sin(ref_heading_) * position.y) / radius_north_ * 180.0 / M_PI;
  gps_message_.longitude = ref_lon_ - (-sin(ref_heading_) * position.x + cos(ref_heading_) * position.y) / radius_east_ * 180.0 / M_PI;
  gps_message_.altitude = ref_alt_ + position.z;

  // Fill GPS message
  gps_message_.header.seq = gps_sequence_;
  gps_message_.header.stamp.sec = current_time.sec;
  gps_message_.header.stamp.nsec = current_time.nsec;

  // Publish the message
  gps_pub_.publish(gps_message_);
  gps_sequence_++;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGpsPlugin);
}
