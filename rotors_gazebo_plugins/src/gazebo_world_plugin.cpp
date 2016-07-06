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

#include "rotors_gazebo_plugins/gazebo_world_plugin.h"

namespace gazebo {

GazeboWorldPlugin::GazeboWorldPlugin()
    : WorldPlugin(),
      ros_node_(NULL) {}

GazeboWorldPlugin::~GazeboWorldPlugin() {
  if (ros_node_) {
    ros_node_->shutdown();
    delete ros_node_;
  }

  if (it_node_)
    delete it_node_;
}
;

void GazeboWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
  world_ = _world;

  // Set the simulation time to current wall time
  ros::Time current_ros_time = ros::Time::now();
  common::Time new_sim_time(current_ros_time.sec, current_ros_time.nsec);
  world_->SetSimTime(new_sim_time);

  std::cout << "PAVEL - setting sim time" << std::endl;
  std::cout << current_ros_time.sec << std::endl;
  std::cout << current_ros_time.nsec << std::endl;

  /*if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_world_plugin] Please specify a robotNamespace.\n";
  ros_node_ = new ros::NodeHandle(namespace_);
  it_node_ = new image_transport::ImageTransport(*ros_node_);

  double frame_rate;
  getSdfParam<double>(_sdf, "frameRate", frame_rate, kDefaultFrameRate);
  getSdfParam<std::string>(_sdf, "imgFrameName", img_frame_name_, kDefaultImgFrameName);

  frame_interval_ = 1.0 / frame_rate;
  last_frame_pub_time_ = ros::Time::now().toSec();

  this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboWorldPlugin::WaitForSceneToLoad, this));

  camera_element_ = _sdf->GetElement("camera");
  if (!camera_element_)
    gzerr << "[gazebo_world_plugin] Please specify a camera element.\n";*/
}

/*void GazeboWorldPlugin::OnRenderUpdate() {
  ros::Time current_time = ros::Time::now();

  if ((current_time.toSec() - last_frame_pub_time_) >= frame_interval_) {
    last_frame_pub_time_ = current_time.toSec();

#if GAZEBO_MAJOR_VERSION > 6
    PublishImageData(cam_->GetImageData(0), current_time);
#else
    PublishImageData(cam_->GetImageData(0), current_time);
#endif
  }
}

void GazeboWorldPlugin::WaitForSceneToLoad() {
  scene_ = rendering::get_scene(world_->GetName());

  if (!scene_) {
    scene_ = rendering::create_scene(world_->GetName(), false, true);

    if (!scene_)
      gzerr << "Unable to create a camera sensor.\n";
  }

  if (scene_ && !cam_) {
    cam_ = scene_->CreateCamera("head", false);

    if (cam_) {
      cam_->SetCaptureData(true);
      cam_->Load(camera_element_);
      cam_->Init();

      /*std::cout << cam_->ImageWidth() << std::endl;
      std::cout << cam_->ImageHeight() << std::endl;

#if GAZEBO_MAJOR_VERSION > 6
      cam_params_.format = cam_->ImageFormat();
      cam_params_.depth = cam_->ImageDepth();
      cam_params_.height = cam_->ImageHeight();
      cam_params_.width = cam_->ImageWidth();
#else
      cam_params_.format = cam_->GetImageFormat();
      cam_params_.depth = cam_->GetImageDepth();
      cam_params_.height = cam_->GetImageHeight();
      cam_params_.width = cam_->GetImageWidth();
#endif

      image_pub_ = it_node_->advertise("view_cam", 1, true);

      //cam_->SetRenderRate(30.0);
      cam_->CreateRenderTexture("head_RTT");
      //cam_->SetWorldPosition(math::Vector3(0.0, 0.0, 5.0));

      std::cout << std::endl;
      std::cout << cam_->Initialized() << std::endl;
      std::cout << std::endl;

      event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

      this->update_connection_ =
        event::Events::ConnectRender(
          boost::bind(&GazeboWorldPlugin::OnRenderUpdate, this));
    }
  }
}

void GazeboWorldPlugin::PublishImageData(const unsigned char* img_data, ros::Time time) {
  std::cout << std::endl;
  std::cout << "test" << std::endl;
  std::cout << cam_->CaptureData() << std::endl;
  std::cout << cam_->Initialized() << std::endl;
  std::cout << cam_->ImageByteSize() << std::endl;
  std::cout << cam_->RenderRate() << std::endl;

  // Fill the image header
  image_msg_.header.frame_id = img_frame_name_;
  image_msg_.header.stamp.sec = time.sec;
  image_msg_.header.stamp.nsec = time.nsec;

  if (img_data) {
    std::cout << "----- test -----" << std::endl;
    fillImage(image_msg_, sensor_msgs::image_encodings::RGB8, cam_params_.height, cam_params_.width,
              cam_params_.depth * cam_params_.width, reinterpret_cast<const void*>(img_data));

    image_pub_.publish(image_msg_);
  }
}*/

GZ_REGISTER_WORLD_PLUGIN(GazeboWorldPlugin);
}
