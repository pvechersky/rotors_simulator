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

#include <rotors_gazebo_gui_plugins/gazebo_gui_rotors_toolbar_plugin.h>

namespace gazebo {

GazeboGuiRotorsToolbarPlugin::GazeboGuiRotorsToolbarPlugin():
    GUIPlugin() {
  this->main_window_ = gui::get_main_window();
  toolbar_ = this->main_window_->GetRenderWidget()->GetToolbar();

  point_coordinates_action_ = new QAction(QIcon("/home/pavel/thesis_ws/src/rotors_simulator/rotors_gazebo_gui_plugins/images/point_coordinates.png"), tr("TEST"), this);
  toolbar_->addAction(point_coordinates_action_);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  QPushButton *forward_button = new QPushButton(tr("Forward"));
  mainLayout->addWidget(forward_button);
  setLayout(mainLayout);
  resize(0, 0);
}

GazeboGuiRotorsToolbarPlugin::~GazeboGuiRotorsToolbarPlugin() {
}

void GazeboGuiRotorsToolbarPlugin::Load(sdf::ElementPtr _sdf) {

}

void GazeboGuiRotorsToolbarPlugin::OnForwardButton() {

}

void GazeboGuiRotorsToolbarPlugin::OnUpdate() {

}

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GazeboGuiRotorsToolbarPlugin);
}

