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

namespace gazebo {

GazeboGpsPlugin::GazeboGpsPlugin()
    : SensorPlugin(),
      node_handle_(0) {}

GazeboGpsPlugin::~GazeboGpsPlugin() {
  this->parent_sensor_->DisconnectUpdated(this->updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboGpsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  // Store the pointer to the parent sensor
  parent_sensor_ = std::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);

  // Retrieve the necessary parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a linkName.\n";

  frame_id_ = link_name_;

  getSdfParam<std::string>(_sdf, "gpsTopic", gps_topic_,
                           mav_msgs::default_topics::GPS);
  getSdfParam<std::string>(_sdf, "groundSpeedTopic", ground_speed_topic_,
                           kDefaultGroundSpeedPubTopic);

  // Connect to the sensor update event.
  this->updateConnection_ =
      this->parent_sensor_->ConnectUpdated(
          boost::bind(&GazeboGpsPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  parent_sensor_->SetActive(true);

  // Initialize the ROS publisher for sending gps location and ground speed
  gps_pub_ = node_handle_->advertise<sensor_msgs::NavSatFix>(gps_topic_, 1);
  ground_speed_pub_ =
          node_handle_->advertise<geometry_msgs::Vector3Stamped>(ground_speed_topic_, 1);

  // Get the sensor noise in case it is known.
  /*u_int8_t position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;

  double std_dev_alt = 0.0;
  double std_dev_lat = 0.0;
  double std_dev_lon = 0.0;

  sensors::NoisePtr alt_noise = this->parent_sensor_->Noise(sensors::SensorNoiseType::GPS_POSITION_ALTITUDE_NOISE_METERS);
  sensors::NoisePtr lat_noise = this->parent_sensor_->Noise(sensors::SensorNoiseType::GPS_POSITION_LATITUDE_NOISE_METERS);
  sensors::NoisePtr lon_noise = this->parent_sensor_->Noise(sensors::SensorNoiseType::GPS_POSITION_LONGITUDE_NOISE_METERS);

  if (alt_noise->GetNoiseType() == sensors::Noise::GAUSSIAN) {
    sensors::GaussianNoiseModelPtr alt_gaussian_noise =
        std::dynamic_pointer_cast<sensors::GaussianNoiseModel>(alt_noise);
    std_dev_alt = alt_gaussian_noise->GetStdDev();
    position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  }

  if (lat_noise->GetNoiseType() == sensors::Noise::GAUSSIAN) {
    sensors::GaussianNoiseModelPtr lat_gaussian_noise =
        std::dynamic_pointer_cast<sensors::GaussianNoiseModel>(lat_noise);
    std_dev_lat = lat_gaussian_noise->GetStdDev();
    position_covariance_type = position_covariance_type & sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  }

  if (lon_noise->GetNoiseType() == sensors::Noise::GAUSSIAN) {
    sensors::GaussianNoiseModelPtr lon_gaussian_noise =
        std::dynamic_pointer_cast<sensors::GaussianNoiseModel>(lon_noise);
    std_dev_lon = lon_gaussian_noise->GetStdDev();
    position_covariance_type = position_covariance_type & sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  }*/

  // Fill the GPS message.
  gps_message_.header.frame_id = frame_id_;
  gps_message_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gps_message_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  //gps_message_.position_covariance[0] = std_dev_lon * std_dev_lon;
  //gps_message_.position_covariance[4] = std_dev_lat * std_dev_lat;
  //gps_message_.position_covariance[8] = std_dev_alt * std_dev_alt;
  //gps_message_.position_covariance_type = position_covariance_type;
}

void GazeboGpsPlugin::OnUpdate() {
  // Get the time of the last measurement
  common::Time current_time;
  current_time = parent_sensor_->LastMeasurementTime();

  // Fill the GPS message.
  gps_message_.latitude = parent_sensor_->Latitude().Degree();
  gps_message_.longitude = parent_sensor_->Longitude().Degree();
  gps_message_.altitude = parent_sensor_->Altitude();
  gps_message_.header.stamp.sec = current_time.sec;
  gps_message_.header.stamp.nsec = current_time.nsec;

  // Publish the GPS message.
  gps_pub_.publish(gps_message_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboGpsPlugin);
}
