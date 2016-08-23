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

GazeboGuiRotorsToolbarPlugin::GazeboGuiRotorsToolbarPlugin() :
    GUIPlugin() {
  QHBoxLayout *mainLayout = new QHBoxLayout;
  QPushButton *forward_button = new QPushButton(tr("Forward"));
  mainLayout->addWidget(forward_button);
  setLayout(mainLayout);
  resize(0, 0);
}

GazeboGuiRotorsToolbarPlugin::~GazeboGuiRotorsToolbarPlugin() {
  delete this->toolbar_;
  this->toolbar_ = NULL;
}

void GazeboGuiRotorsToolbarPlugin::Load(sdf::ElementPtr _sdf) {
  this->toolbar_ = gui::get_main_window()->GetRenderWidget()->GetToolbar();

  this->point_coordinates_action_ = new QAction(QIcon("/home/pavel/thesis_ws/src/rotors_simulator/rotors_gazebo_gui_plugins/images/point_coordinates.png"), tr("TEST"), this);
  this->point_coordinates_action_->setStatusTip(tr("TEST"));
  this->point_coordinates_action_->setCheckable(true);
  this->point_coordinates_action_->setToolTip(tr("TEST"));
  connect(this->point_coordinates_action_, SIGNAL(triggered()), this, SLOT(PointCoordinatesMode()));

  this->view_selection_action_ = new QAction(QIcon("/home/pavel/thesis_ws/src/rotors_simulator/rotors_gazebo_gui_plugins/images/view_selection.png"), tr("TEST"), this);
  this->view_selection_action_->setStatusTip(tr("TEST"));
  this->view_selection_action_->setCheckable(true);
  this->view_selection_action_->setToolTip(tr("TEST"));
  connect(this->view_selection_action_, SIGNAL(triggered()), this, SLOT(SelectView()));

  this->toolbar_->addSeparator();
  this->toolbar_->addAction(this->point_coordinates_action_);
  this->toolbar_->addAction(this->view_selection_action_);
}

void GazeboGuiRotorsToolbarPlugin::PointCoordinatesMode() {
  bool is_checked = this->view_selection_action_->isEnabled();

  if (is_checked) {
    this->point_coordinates_dialog_ = new PointCoordinatesDialog(this);
    this->point_coordinates_dialog_->show();
  }
  else {
    this->point_coordinates_dialog_->hide();
    delete this->point_coordinates_dialog_;
    this->point_coordinates_dialog_ = NULL;
  }
}

void GazeboGuiRotorsToolbarPlugin::SelectView() {

}

/*void GazeboGuiRotorsToolbarPlugin::OnUpdate() {

}*/

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GazeboGuiRotorsToolbarPlugin);
}

