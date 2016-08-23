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

#ifndef ROTORS_GAZEBO_GUI_ROTORS_TOOLBAR_PLUGIN_H
#define ROTORS_GAZEBO_GUI_ROTORS_TOOLBAR_PLUGIN_H

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/rendering.hh>

#include <rotors_gazebo_gui_plugins/point_coordinates_dialog.h>

namespace gazebo {

class GAZEBO_VISIBLE GazeboGuiRotorsToolbarPlugin : public GUIPlugin {
 Q_OBJECT

 public:
  GazeboGuiRotorsToolbarPlugin();
  virtual ~GazeboGuiRotorsToolbarPlugin();

 protected:
  void Load(sdf::ElementPtr _sdf);

 protected slots:
  void PointCoordinatesMode();
  void SelectView();

 private:
  //void OnUpdate();

  //event::ConnectionPtr update_connection_;

  QToolBar *toolbar_;

  QAction *point_coordinates_action_;
  QAction *view_selection_action_;

  PointCoordinatesDialog *point_coordinates_dialog_;
};
}

#endif // ROTORS_GAZEBO_GUI_ROTORS_TOOLBAR_PLUGIN_H
