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
      node_handle_(0) {
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

  xyz_pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>("xyz_gps", 1);

  xyz_msg_.header.frame_id = frame_id_;

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
}

void GazeboGpsPlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();

  // Get the current world pose and velocity from Gazebo
  math::Vector3 position = model_->GetWorldPose().pos;
  math::Vector3 velocity = model_->GetWorldLinearVel();

  // Update the GPS coordinates
  gps_message_.latitude = ref_lat_ + (cos(ref_heading_) * position.x + sin(ref_heading_) * position.y) / radius_north_ * 180.0 / M_PI;
  gps_message_.longitude = ref_lon_ - (-sin(ref_heading_) * position.x + cos(ref_heading_) * position.y) / radius_east_ * 180.0 / M_PI;
  gps_message_.altitude = ref_alt_ + position.z;

  // Fill the GPS message
  gps_message_.header.stamp.sec = current_time.sec;
  gps_message_.header.stamp.nsec = current_time.nsec;

  // Publish the GPS message
  gps_pub_.publish(gps_message_);

  // Update the ground speed message
  ground_speed_msg_.header.stamp.sec = current_time.sec;
  ground_speed_msg_.header.stamp.nsec = current_time.nsec;
  ground_speed_msg_.twist.linear.x = velocity.x;
  ground_speed_msg_.twist.linear.y = velocity.y;
  ground_speed_msg_.twist.linear.z = velocity.z;

  // Publish the ground speed message
  ground_speed_pub_.publish(ground_speed_msg_);

  xyz_msg_.header.stamp.sec = current_time.sec;
  xyz_msg_.header.stamp.nsec = current_time.nsec;

  double x;
  double y;
  UTM::UTM(gps_message_.latitude, gps_message_.longitude, &x, &y);

  xyz_msg_.pose.position.x = (y - start_northing_);
  xyz_msg_.pose.position.y = -(x - start_easting_);

  xyz_pub_.publish(xyz_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGpsPlugin);
}

/*#include "rotors_gazebo_plugins/gazebo_gps_plugin.h"

namespace gazebo {

GazeboGpsPlugin::GazeboGpsPlugin()
    : SensorPlugin(),
      node_handle_(0),
      random_generator_(random_device_()) {}

GazeboGpsPlugin::~GazeboGpsPlugin() {
  this->parent_sensor_->DisconnectUpdated(this->updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboGpsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  // Store the pointer to the parent sensor.
#if GAZEBO_MAJOR_VERSION > 6
  parent_sensor_ = std::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);
  world_ = physics::get_world(parent_sensor_->WorldName());
#else
  parent_sensor_ = boost::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);
  world_ = physics::get_world(parent_sensor_->GetWorldName());
#endif

  // Retrieve the necessary parameters.
  std::string node_namespace;
  std::string link_name;

  if (_sdf->HasElement("robotNamespace"))
    node_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(node_namespace);

  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a linkName.\n";

  std::string frame_id = link_name;

  // Get the pointer to the link that holds the sensor.
  link_ = boost::dynamic_pointer_cast<physics::Link>(world_->GetByName(link_name));
  if (link_ == NULL)
    gzerr << "[gazebo_gps_plugin] Couldn't find specified link \"" << link_name << "\"\n";

  // Retrieve the rest of the SDF parameters.
  double hor_pos_std_dev;
  double ver_pos_std_dev;
  double hor_vel_std_dev;
  double ver_vel_std_dev;

  getSdfParam<std::string>(_sdf, "gpsTopic", gps_topic_,
                           mav_msgs::default_topics::GPS);
  getSdfParam<std::string>(_sdf, "groundSpeedTopic", ground_speed_topic_,
                           kDefaultGroundSpeedPubTopic);
  getSdfParam<double>(_sdf, "horPosStdDev", hor_pos_std_dev, kDefaultHorPosStdDev);
  getSdfParam<double>(_sdf, "verPosStdDev", ver_pos_std_dev, kDefaultVerPosStdDev);
  getSdfParam<double>(_sdf, "horVelStdDev", hor_vel_std_dev, kDefaultHorVelStdDev);
  getSdfParam<double>(_sdf, "verVelStdDev", ver_vel_std_dev, kDefaultVerVelStdDev);

  // Connect to the sensor update event.
  this->updateConnection_ =
      this->parent_sensor_->ConnectUpdated(
          boost::bind(&GazeboGpsPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  parent_sensor_->SetActive(true);

  // Initialize the ROS publisher for sending gps location and ground speed.
  gps_pub_ = node_handle_->advertise<sensor_msgs::NavSatFix>(gps_topic_, 1);
  ground_speed_pub_ = node_handle_->advertise<geometry_msgs::TwistStamped>(ground_speed_topic_, 1);

  // Initialize the normal distributions for ground speed.
  ground_speed_n_[0] = NormalDistribution(0, hor_vel_std_dev);
  ground_speed_n_[1] = NormalDistribution(0, hor_vel_std_dev);
  ground_speed_n_[2] = NormalDistribution(0, ver_vel_std_dev);

  // Fill the GPS message.
  gps_message_.header.frame_id = frame_id;
  gps_message_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gps_message_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  gps_message_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
  gps_message_.position_covariance[0] = hor_pos_std_dev * hor_pos_std_dev;
  gps_message_.position_covariance[4] = hor_pos_std_dev * hor_pos_std_dev;
  gps_message_.position_covariance[8] = ver_pos_std_dev * ver_pos_std_dev;

  // Fill the ground speed message.
  ground_speed_message_.header.frame_id = frame_id;
}

void GazeboGpsPlugin::OnUpdate() {
  // Get the time of the last measurement.
  common::Time current_time;

  // Get the linear velocity in the world frame.
  math::Vector3 W_ground_speed_W_L = link_->GetWorldLinearVel();

  // Apply noise to ground speed.
  W_ground_speed_W_L += math::Vector3(ground_speed_n_[0](random_generator_),
          ground_speed_n_[1](random_generator_),
          ground_speed_n_[2](random_generator_));

  // Fill the GPS message.
#if GAZEBO_MAJOR_VERSION > 6
  current_time = parent_sensor_->LastMeasurementTime();

  gps_message_.latitude = parent_sensor_->Latitude().Degree();
  gps_message_.longitude = parent_sensor_->Longitude().Degree();
  gps_message_.altitude = parent_sensor_->Altitude();
#else
  current_time = parent_sensor_->GetLastMeasurementTime();

  gps_message_.latitude = parent_sensor_->GetLatitude().Degree();
  gps_message_.longitude = parent_sensor_->GetLongitude().Degree();
  gps_message_.altitude = parent_sensor_->GetAltitude();
#endif
  gps_message_.header.stamp.sec = current_time.sec;
  gps_message_.header.stamp.nsec = current_time.nsec;

  // Fill the ground speed message.
  ground_speed_message_.twist.linear.x = W_ground_speed_W_L.x;
  ground_speed_message_.twist.linear.y = W_ground_speed_W_L.y;
  ground_speed_message_.twist.linear.z = W_ground_speed_W_L.z;
  ground_speed_message_.header.stamp.sec = current_time.sec;
  ground_speed_message_.header.stamp.nsec = current_time.nsec;

  // Publish the GPS message.
  gps_pub_.publish(gps_message_);

  // Publish the ground speed message.
  ground_speed_pub_.publish(ground_speed_message_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboGpsPlugin);
}*/
