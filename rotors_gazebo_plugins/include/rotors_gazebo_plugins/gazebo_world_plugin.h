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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WORLD_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WORLD_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Defaults
static constexpr double kDefaultFrameRate = 30.0;
static const std::string kDefaultImgFrameName = "world_cam";

struct CamParams {
  std::string format;
  unsigned int height;
  unsigned int width;
  unsigned int depth;
};

class GazeboWorldPlugin : public WorldPlugin {
 public:
  GazeboWorldPlugin();
  virtual ~GazeboWorldPlugin();

 protected:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  void OnRenderUpdate();

  void WaitForSceneToLoad();

  void PublishImageData(const unsigned char* img_data, ros::Time time);

 private:
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;

  sdf::ElementPtr camera_element_;

  std::string namespace_;
  std::string img_frame_name_;

  rendering::ScenePtr scene_;
  rendering::CameraPtr cam_;

  CamParams cam_params_;

  ros::NodeHandle* ros_node_;
  image_transport::ImageTransport* it_node_;
  image_transport::Publisher image_pub_;

  sensor_msgs::Image image_msg_;

  double frame_interval_;
  double last_frame_pub_time_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WORLD_PLUGIN_H
