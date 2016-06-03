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

#ifndef ROTORS_GAZEBO_PLUGINS_VIEW_CONTROL_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_VIEW_CONTROL_PLUGIN_H

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/rendering.hh>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo
{
// Constants
static const math::Vector3 kChaseCamOffset(-10.0, 0.0, 2.0);
static const math::Vector3 kForwardCamOffset(0.5, 0.0, 0.5);
static const std::string kFrameName = "user_cam";

// Defaults
static constexpr double kDefaultFrameRate = 10.0;
static constexpr bool kDefaultPublishImageData = false;

class GAZEBO_VISIBLE GazeboViewControlPlugin : public GUIPlugin {
 Q_OBJECT

 public:
  GazeboViewControlPlugin();
  virtual ~GazeboViewControlPlugin();

  void PublishImageData(const unsigned char* img_data, ros::Time time);

 protected:
  void Load(sdf::ElementPtr _sdf);

 protected slots:
  void OnForwardButton();
  void OnChaseButton();
  void OnStopButton();

 private:
  void OnUpdate();

  bool is_tracking_;
  bool publish_image_data_;

  double frame_interval_;
  double last_frame_pub_time_;

  event::ConnectionPtr update_connection_;

  math::Vector3 cam_offset_;

  rendering::UserCameraPtr user_cam_;

  // Pointer to the visual of the object we want to track
  rendering::VisualPtr visual_;

  ros::NodeHandle* ros_node_;
  image_transport::ImageTransport* it_node_;
  image_transport::Publisher image_pub_;

  sensor_msgs::Image image_msg_;

  // Image parameters
  std::string image_format_;
  unsigned int image_height_;
  unsigned int image_width_;
  unsigned int image_depth_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_VIEW_CONTROL_PLUGIN_H
