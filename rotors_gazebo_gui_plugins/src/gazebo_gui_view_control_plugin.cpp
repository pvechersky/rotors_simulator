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

#include <rotors_gazebo_gui_plugins/gazebo_gui_view_control_plugin.h>

namespace gazebo {

GazeboGuiViewControlPlugin::GazeboGuiViewControlPlugin():
    GUIPlugin(),
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

GazeboGuiViewControlPlugin::~GazeboGuiViewControlPlugin() {
}

void GazeboGuiViewControlPlugin::Load(sdf::ElementPtr _sdf) {
  // Get a pointer to the active user camera and the scene
  user_cam_ = gui::get_active_camera();

#if GAZEBO_MAJOR_VERSION > 6
  cam_offset_ = user_cam_->WorldPosition();
#else
  cam_offset_ = user_cam_->GetWorldPosition();
#endif

  this->update_connection_ =
      event::Events::ConnectRender(
          boost::bind(&GazeboGuiViewControlPlugin::OnUpdate, this));
}

void GazeboGuiViewControlPlugin::OnForwardButton() {
  is_tracking_ = true;
  cam_offset_ = kForwardCamOffset;
}

void GazeboGuiViewControlPlugin::OnChaseButton() {
  is_tracking_ = true;
  cam_offset_ = kChaseCamOffset;
}

void GazeboGuiViewControlPlugin::OnStopButton() {
  is_tracking_ = false;
}

void GazeboGuiViewControlPlugin::OnUpdate() {
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
    new_cam_pose.rot.w = visual_pose.rot.w;
    new_cam_pose.rot.x = visual_pose.rot.x;
    new_cam_pose.rot.y = visual_pose.rot.y;
    new_cam_pose.rot.z = visual_pose.rot.z;

    // Rotate the camera position offset vector into the visual's body frame
    math::Vector3 cam_offset_body_ = visual_pose.rot.RotateVector(cam_offset_);

    // Translate the position of the camera
    new_cam_pose.pos.x = visual_pose.pos.x + cam_offset_body_.x;
    new_cam_pose.pos.y = visual_pose.pos.y + cam_offset_body_.y;
    new_cam_pose.pos.z = visual_pose.pos.z + cam_offset_body_.z;

    // Set the new camera pose
    user_cam_->SetWorldPose(new_cam_pose);
  }
}

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GazeboGuiViewControlPlugin);
}

