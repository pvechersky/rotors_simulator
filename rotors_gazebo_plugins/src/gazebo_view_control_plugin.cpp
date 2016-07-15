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

#include <rotors_gazebo_plugins/gazebo_view_control_plugin.h>

namespace gazebo {

GazeboViewControlPlugin::GazeboViewControlPlugin():
    GUIPlugin(),
    ros_node_(0),
    is_tracking_(false) {
  // Set the frame background and foreground colors
  setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 0); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *forward_button = new QPushButton(tr("Forward"));
  QPushButton *chase_button = new QPushButton(tr("Chase"));
  QPushButton *stop_button = new QPushButton(tr("Stop Tracking"));
  connect(forward_button, SIGNAL(clicked()), this, SLOT(OnForwardButton()));
  connect(chase_button, SIGNAL(clicked()), this, SLOT(OnChaseButton()));
  connect(stop_button, SIGNAL(clicked()), this, SLOT(OnStopButton()));

  // Add the buttons to the frame's layout
  frameLayout->addWidget(forward_button);
  frameLayout->addWidget(chase_button);
  frameLayout->addWidget(stop_button);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  setLayout(mainLayout);

  // Position and resize this widget
  move(10, 10);
  resize(120, 90);
}

GazeboViewControlPlugin::~GazeboViewControlPlugin() {
  if (ros_node_) {
    ros_node_->shutdown();
    delete ros_node_;
  }
  if (it_node_) {
    delete it_node_;
  }
}

void GazeboViewControlPlugin::Load(sdf::ElementPtr _sdf) {
  int n = 1;
  char * c[1];
  c[1] = "gazebo";
  ros::init(n, &c[0], "pavel");
  std::string namespace_ = "techpod";
  ros_node_ = new ros::NodeHandle(namespace_);
  it_node_ = new image_transport::ImageTransport(*ros_node_);

  // Get a pointer to the active user camera and the scene
  user_cam_ = gui::get_active_camera();

#if GAZEBO_MAJOR_VERSION > 6
  cam_offset_ = user_cam_->WorldPosition();
#else
  cam_offset_ = user_cam_->GetWorldPosition();
#endif

  if (publish_image_data_) {
    double frame_rate;
    getSdfParam<double>(_sdf, "frameRate", frame_rate, kDefaultFrameRate);
    getSdfParam<bool>(_sdf, "publishImageData", publish_image_data_, kDefaultPublishImageData);

    frame_interval_ = 1.0 / frame_rate;
    last_frame_pub_time_ = ros::Time::now().toSec();

#if GAZEBO_MAJOR_VERSION > 6
    image_format_ = user_cam_->ImageFormat();
    image_depth_ = user_cam_->ImageDepth();
#else
    image_format_ = user_cam_->GetImageFormat();
    image_depth_ = user_cam_->GetImageDepth();
#endif

    image_height_ = user_cam_->GetImageHeight();
    image_width_ = user_cam_->GetImageWidth();

    image_pub_ = it_node_->advertise("view_cam", 1, true);

    user_cam_->SetCaptureData(true);
  }

  this->update_connection_ =
      event::Events::ConnectRender(
          boost::bind(&GazeboViewControlPlugin::OnUpdate, this));

}

void GazeboViewControlPlugin::OnForwardButton() {
  is_tracking_ = true;
  cam_offset_ = kForwardCamOffset;
}

void GazeboViewControlPlugin::OnChaseButton() {
  is_tracking_ = true;
  cam_offset_ = kChaseCamOffset;
}

void GazeboViewControlPlugin::OnStopButton() {
  is_tracking_ = false;
}

void GazeboViewControlPlugin::OnUpdate() {
  // If it has not been loaded already, attempt to get a handle to the visual
  // we want to track
  if (!visual_) {
    rendering::ScenePtr scene = rendering::get_scene();
    visual_ = scene->GetVisual("techpod::techpod/base_link");
    return;
  }

  if (is_tracking_) {
    // Get the world pose of the visual we are tracking
    math::Pose visual_pose = visual_->GetWorldPose();

    // Align the camera with the orientation of the visual
    math::Pose new_cam_pose;
    math::Quaternion body_orientation(M_PI, 0.0, 0.0);
    math::Quaternion cam_orientation = body_orientation * visual_pose.rot;
    /*new_cam_pose.rot.w = visual_pose.rot.w;
    new_cam_pose.rot.x = visual_pose.rot.x;
    new_cam_pose.rot.y = visual_pose.rot.y;
    new_cam_pose.rot.z = visual_pose.rot.z;*/
    new_cam_pose.rot.w = cam_orientation.w;
    new_cam_pose.rot.x = cam_orientation.x;
    new_cam_pose.rot.y = cam_orientation.y;
    new_cam_pose.rot.z = cam_orientation.z;

    // Rotate the camera position offset vector into the visual's body frame
    math::Vector3 cam_offset_body_ = visual_pose.rot.RotateVector(cam_offset_);

    // Translate the position of the camera
    new_cam_pose.pos.x = visual_pose.pos.x + cam_offset_body_.x;
    new_cam_pose.pos.y = visual_pose.pos.y + cam_offset_body_.y;
    new_cam_pose.pos.z = visual_pose.pos.z + cam_offset_body_.z;

    // Set the new camera pose
    user_cam_->SetWorldPose(new_cam_pose);
  }

  if (publish_image_data_) {
    ros::Time current_time = ros::Time::now();
    if ((current_time.toSec() - last_frame_pub_time_) >= frame_interval_) {
      last_frame_pub_time_ = current_time.toSec();

#if GAZEBO_MAJOR_VERSION > 6
      PublishImageData(user_cam_->ImageData(0), current_time);
#else
      PublishImageData(user_cam_->GetImageData(0), current_time);
#endif
    }
  }
}

void GazeboViewControlPlugin::PublishImageData(const unsigned char* img_data, ros::Time time) {
  // Fill the image header
  image_msg_.header.frame_id = kFrameName;
  image_msg_.header.stamp.sec = time.sec;
  image_msg_.header.stamp.nsec = time.nsec;

  if (img_data) {
    fillImage(image_msg_, sensor_msgs::image_encodings::RGB8, image_height_, image_width_,
              image_depth_ * image_width_, reinterpret_cast<const void*>(img_data));

    image_pub_.publish(image_msg_);
  }
}

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GazeboViewControlPlugin);
}

