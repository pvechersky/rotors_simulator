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

#ifndef ROTORS_GAZEBO_GUI_POINT_COORDINATES_DIALOG_H
#define ROTORS_GAZEBO_GUI_POINT_COORDINATES_DIALOG_H

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gui/gui.hh>
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo {

class GAZEBO_VISIBLE PointCoordinatesDialog : public QDialog {
 Q_OBJECT

 public:
  PointCoordinatesDialog(QWidget *_parent = 0);
  virtual ~PointCoordinatesDialog();
};
}

#endif // ROTORS_GAZEBO_GUI_POINT_COORDINATES_DIALOG_H
