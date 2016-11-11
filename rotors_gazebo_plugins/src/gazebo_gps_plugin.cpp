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
      random_generator_() {
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

  std::string gps_topic;
  std::string ground_speed_topic;

  // Retrieve the rest of the SDF parameters
  getSdfParam<std::string>(_sdf, "gpsTopic", gps_topic, mav_msgs::default_topics::GPS);
  getSdfParam<std::string>(_sdf, "groundSpeedTopic", ground_speed_topic, kDefaultGroundSpeedPubTopic);
  getSdfParam<double>(_sdf, "referenceLatitude", ref_lat_, kDefaultRefLat);
  getSdfParam<double>(_sdf, "referenceLongitude", ref_lon_, kDefaultRefLon);
  getSdfParam<double>(_sdf, "referenceAltitude", ref_alt_, kDefaultRefAlt);
  getSdfParam<double>(_sdf, "referenceHeading", ref_heading_, kDefaultRefHeading);

  UTM::UTM(ref_lat_, ref_lon_, &start_easting_, &start_northing_);

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboGpsPlugin::OnUpdate, this, _1));

  gps_pub_ = node_handle_->advertise<sensor_msgs::NavSatFix>(gps_topic, 1);
  ground_speed_pub_ = node_handle_->advertise<geometry_msgs::TwistStamped>(ground_speed_topic, 1);

  // Fill the GPS message
  gps_message_.header.frame_id = frame_id_;
  gps_message_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gps_message_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  gps_message_.latitude = ref_lat_;
  gps_message_.longitude = ref_lon_;
  gps_message_.altitude = ref_alt_;
  gps_message_.position_covariance.fill(0.0);

  // Fill the ground speed message
  ground_speed_msg_.header.frame_id = frame_id_;

  double temp = 1.0 / (1.0 - kEccentrity2 * sin(ref_lat_ * M_PI / 180.0) * sin(ref_lat_ * M_PI / 180.0));
  double prime_vertical_radius = kEquatorialRadius * sqrt(temp);
  radius_north_ = prime_vertical_radius * (1 - kEccentrity2) * temp;
  radius_east_  = prime_vertical_radius * cos(ref_lat_ * M_PI / 180.0);
  ref_heading_ = ref_heading_ * M_PI / 180.0;

  last_time_ = world_->GetSimTime();

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  double sigma_bon_p = 0.1; //position_turn_on_bias_sigma;
  double sigma_bon_v = 0.01; //velocity_turn_on_bias_sigma;
  for (int i = 0; i < 3; ++i) {
    position_turn_on_bias_[i] =
        sigma_bon_p * standard_normal_distribution_(random_generator_);
    velocity_turn_on_bias_[i] =
        sigma_bon_v * standard_normal_distribution_(random_generator_);
  }

  position_bias_.setZero();
  velocity_bias_.setZero();
}

void GazeboGpsPlugin::AddNoise(Eigen::Vector3d* position,
                               Eigen::Vector3d* velocity,
                               const double dt) {
  ROS_ASSERT(position != nullptr);
  ROS_ASSERT(velocity != nullptr);
  ROS_ASSERT(dt > 0.0);

  // Position
  double tau_p = 300.0; //position_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_p_d = 1 / sqrt(dt) * 0.5; //position_noise_density;
  double sigma_b_p = 0.2; //position_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_p_d =
      sqrt( - sigma_b_p * sigma_b_p * tau_p / 2.0 *
      (exp(-2.0 * dt / tau_p) - 1.0));
  // Compute state-transition.
  double phi_p_d = exp(-1.0 / tau_p * dt);
  // Simulate position noise processes and add them to the true positon
  for (int i = 0; i < 3; ++i) {
    position_bias_[i] = phi_p_d * position_bias_[i] +
        sigma_b_p_d * standard_normal_distribution_(random_generator_);
    (*position)[i] = (*position)[i] +
        position_bias_[i] +
        sigma_p_d * standard_normal_distribution_(random_generator_) +
        position_turn_on_bias_[i];
  }

  // Velocity
  double tau_v = 300; //velocity_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_v_d = 1 / sqrt(dt) * 0.002; //velocity_noise_density;
  double sigma_b_v = 0.0025; //velocity_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_v_d =
      sqrt( - sigma_b_v * sigma_b_v * tau_v / 2.0 *
      (exp(-2.0 * dt / tau_v) - 1.0));
  // Compute state-transition.
  double phi_v_d = exp(-1.0 / tau_v * dt);
  // Simulate velocity noise processes and add them to the true velocity
  for (int i = 0; i < 3; ++i) {
    velocity_bias_[i] = phi_v_d * velocity_bias_[i] +
        sigma_b_v_d * standard_normal_distribution_(random_generator_);
    (*velocity)[i] = (*velocity)[i] +
        velocity_bias_[i] +
        sigma_v_d * standard_normal_distribution_(random_generator_) +
        velocity_turn_on_bias_[i];
  }
}

void GazeboGpsPlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;

  // Get the current world pose and velocity from Gazebo
  math::Vector3 position = model_->GetWorldPose().pos;
  math::Vector3 velocity = model_->GetWorldLinearVel();

  Eigen::Vector3d pos(position.x, position.y, position.z);
  Eigen::Vector3d vel(velocity.x, velocity.y, velocity.z);

  //AddNoise(&pos, &vel, dt);

  // Update the GPS coordinates
  //gps_message_.latitude = ref_lat_ + (cos(ref_heading_) * position.x + sin(ref_heading_) * position.y) / radius_north_ * 180.0 / M_PI;
  //gps_message_.longitude = ref_lon_ - (-sin(ref_heading_) * position.x + cos(ref_heading_) * position.y) / radius_east_ * 180.0 / M_PI;
  //gps_message_.altitude = ref_alt_ + position.z;

  gps_message_.latitude = ref_lat_ + (cos(ref_heading_) * pos.x() + sin(ref_heading_) * pos.y()) / radius_north_ * 180.0 / M_PI;
  gps_message_.longitude = ref_lon_ - (-sin(ref_heading_) * pos.x() + cos(ref_heading_) * pos.y()) / radius_east_ * 180.0 / M_PI;
  gps_message_.altitude = ref_alt_ + pos.z();

  // Fill the GPS message
  gps_message_.header.stamp.sec = current_time.sec;
  gps_message_.header.stamp.nsec = current_time.nsec;

  // Publish the GPS message
  gps_pub_.publish(gps_message_);

  // Update the ground speed message
  ground_speed_msg_.header.stamp.sec = current_time.sec;
  ground_speed_msg_.header.stamp.nsec = current_time.nsec;
  //ground_speed_msg_.twist.linear.x = velocity.x;
  //ground_speed_msg_.twist.linear.y = velocity.y;
  //ground_speed_msg_.twist.linear.z = velocity.z;
  ground_speed_msg_.twist.linear.x = vel.x();
  ground_speed_msg_.twist.linear.y = vel.y();
  ground_speed_msg_.twist.linear.z = vel.z();

  // Publish the ground speed message
  ground_speed_pub_.publish(ground_speed_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGpsPlugin);
}
