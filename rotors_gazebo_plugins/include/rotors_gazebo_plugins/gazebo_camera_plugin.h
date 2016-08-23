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

#ifndef ROTORS_GAZEBO_PLUGINS_CAMERA_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_CAMERA_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static const std::string kDefaultImagePubTopic = "camera";

class GazeboCameraPlugin : public SensorPlugin {
 public:
  GazeboCameraPlugin();
  virtual ~GazeboCameraPlugin();

 protected:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  void OnUpdate();

 private:
  // ROS interface
  ros::NodeHandle* node_handle_;
  ros::Publisher image_pub_;
  std::string image_topic_;

  sensor_msgs::Image image_msg_;

  std::string type_;
  int width_;
  int height_;
  int skip_;

  // Pointer to the parent sensor
  sensors::CameraSensorPtr parent_sensor_;

  // Pointer to the world
  physics::WorldPtr world_;

  // Pointer to the sensor link
  physics::LinkPtr link_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_CAMERA_PLUGIN_H
