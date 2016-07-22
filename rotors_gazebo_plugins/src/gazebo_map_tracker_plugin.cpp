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

#include <rotors_gazebo_plugins/gazebo_map_tracker_plugin.h>

namespace gazebo {

GazeboMapTrackerPlugin::GazeboMapTrackerPlugin():
    GUIPlugin() {
}

GazeboMapTrackerPlugin::~GazeboMapTrackerPlugin() {
}

void GazeboMapTrackerPlugin::Load(sdf::ElementPtr _sdf) {
  // Set the frame background and foreground colors
  setStyleSheet("QFrame { background-color : rgba(0, 0, 0, 0); color : white; }");

  // Create the main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;

  // Create a graphics scene object
  QGraphicsScene *scene = new QGraphicsScene();
  QGraphicsPixmapItem *item = new QGraphicsPixmapItem(QPixmap("/home/pavel/yosemite.png"));
  item->setScale(2.0);
  item->setRotation(-90.0);
  scene->addItem(item);

  // Create a graphics view object
  QGraphicsView *view = new QGraphicsView(scene);

  // Add the frame to the main layout
  mainLayout->addWidget(view);

  // Remove margins to reduce space
  mainLayout->setContentsMargins(0, 0, 0, 0);

  setLayout(mainLayout);

  // Position and resize this widget
  move(950, 480);
  resize(270, 270);
}

void GazeboMapTrackerPlugin::OnUpdate() {
}

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GazeboMapTrackerPlugin);
}
