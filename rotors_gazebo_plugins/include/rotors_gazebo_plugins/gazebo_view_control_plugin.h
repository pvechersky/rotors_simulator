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

#ifndef ROTORS_GAZEBO_PLUGINS_VIEW_CONTROL_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_VIEW_CONTROL_PLUGIN_H

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/MouseEvent.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/rendering_force.h"
//#include "rotors_gazebo_plugins/rendering_torque.h"

namespace gazebo
{
// Constants
static constexpr double kWaitTime = 3.0;
static const math::Vector3 kChaseCamOffset(-3.0, 0.0, 0.5);
static const math::Vector3 kForwardCamOffset(0.5, 0.0, -0.2);
static const math::Vector3 kOrthogonalCamOffset(-3.0, -2.0, 1.0);
static const math::Vector3 kOrthogonalRotation(0.0, 0.27, 0.588);
static const math::Vector3 kCameraCamOffsetStart(-3.0, -2.0, 1.0);
static const math::Vector3 kCameraCamOffsetGoal(-0.15, -0.1, 0.05);
static const math::Vector3 kCameraCamOffsetSecondGoal(0.02, -0.05, 0.2);
static const math::Vector3 kCameraCamRotationGoal(0.0, M_PI * 0.5, 0.0);

class GAZEBO_VISIBLE GazeboViewControlPlugin : public GUIPlugin {
 Q_OBJECT

 public:
  GazeboViewControlPlugin();
  virtual ~GazeboViewControlPlugin();

  void ForceCallback(ConstVector3dPtr &force_msg);
  //void TorqueCallback(ConstVector3dPtr &torque_msg);

 protected:
  void Load(sdf::ElementPtr _sdf);

 protected slots:
  void OnForwardButton();
  void OnChaseButton();
  void OnOrthogonalButton();
  void OnCameraButton();
  void OnStopButton();

 private:
  void OnUpdate();

  bool OnMousePress(const common::MouseEvent& _event);

  bool is_tracking_;

  event::ConnectionPtr update_connection_;

  math::Vector3 cam_offset_;
  math::Vector3 offset_step_;

  math::Vector3 cam_rotation_;
  math::Vector3 rotation_step_;

  rendering::UserCameraPtr user_cam_;

  // Pointer to the visual of the object we want to track
  rendering::VisualPtr visual_;

  RenderingForcePtr rendering_force_;
  //RenderingTorquePtr rendering_torque_;

  math::Vector3 force_vector_;
  //math::Vector3 torque_vector_;

  transport::NodePtr node_;
  transport::SubscriberPtr force_sub_;
  //transport::SubscriberPtr torque_sub_;

  std::string mode_;

  common::Time timer_start_;

  int num_steps_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_VIEW_CONTROL_PLUGIN_H
