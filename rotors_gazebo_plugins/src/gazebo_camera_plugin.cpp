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

#include "rotors_gazebo_plugins/gazebo_camera_plugin.h"

namespace gazebo {

GazeboCameraPlugin::GazeboCameraPlugin()
    : SensorPlugin(),
      node_handle_(0) {}

GazeboCameraPlugin::~GazeboCameraPlugin() {
  this->parent_sensor_->DisconnectUpdated(this->updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  // Store the pointer to the parent sensor.
#if GAZEBO_MAJOR_VERSION > 6
  parent_sensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
  world_ = physics::get_world(parent_sensor_->WorldName());
#else
  parent_sensor_ = boost::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
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

  // Get the pointer to the link that holds the sensor.
  link_ = boost::dynamic_pointer_cast<physics::Link>(world_->GetByName(link_name));
  if (link_ == NULL)
    gzerr << "[gazebo_gps_plugin] Couldn't find specified link \"" << link_name << "\"\n";

  getSdfParam<std::string>(_sdf, "imageTopic", image_topic_, kDefaultImagePubTopic);

  // Connect to the sensor update event.
  this->updateConnection_ =
      this->parent_sensor_->ConnectUpdated(
          boost::bind(&GazeboCameraPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  parent_sensor_->SetActive(true);

  // Initialize the ROS publisher.
  image_pub_ = node_handle_->advertise<sensor_msgs::Image>(image_topic_, 1);

  // Initialize the image message.
  image_msg_.header.frame_id = link_name;

  type_ = sensor_msgs::image_encodings::MONO8;
  width_ = 752;
  height_ = 480;
  skip_ = 1;
}

void GazeboCameraPlugin::OnUpdate() {
  common::Time last_update_time = parent_sensor_->GetLastUpdateTime();

  image_msg_.header.stamp.sec = last_update_time.sec;
  image_msg_.header.stamp.nsec = last_update_time.nsec;

  // Copy from src to image_msg_.
  sensor_msgs::fillImage(image_msg_, type_, height_, width_, skip_ * width_,
                         reinterpret_cast<const void*>(parent_sensor_->GetImageData()));

  // Publish the new image message.
  image_pub_.publish(image_msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboCameraPlugin);
}
