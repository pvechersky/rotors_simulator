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

#ifndef ROTORS_GAZEBO_GUI_PLUGINS_VIEW_CONTROL_PLUGIN_H
#define ROTORS_GAZEBO_GUI_PLUGINS_VIEW_CONTROL_PLUGIN_H

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/rendering.hh>

namespace gazebo
{
// Constants
static const math::Vector3 kChaseCamOffset(-3.0, 0.0, 0.5);
static const math::Vector3 kForwardCamOffset(0.5, 0.0, -0.2);

class GAZEBO_VISIBLE GazeboGuiViewControlPlugin : public GUIPlugin {
 Q_OBJECT

 public:
  GazeboGuiViewControlPlugin();
  virtual ~GazeboGuiViewControlPlugin();

 protected:
  void Load(sdf::ElementPtr _sdf);

 protected slots:
  void OnForwardButton();
  void OnChaseButton();
  void OnStopButton();

 private:
  void OnUpdate();

  bool is_tracking_;

  event::ConnectionPtr update_connection_;

  math::Vector3 cam_offset_;

  rendering::UserCameraPtr user_cam_;

  // Pointer to the visual of the object we want to track
  rendering::VisualPtr visual_;
};
}

#endif // ROTORS_GAZEBO_GUI_PLUGINS_VIEW_CONTROL_PLUGIN_H
