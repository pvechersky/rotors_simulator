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

#ifndef ROTORS_HIL_CONTROL_INTERFACE_H_
#define ROTORS_HIL_CONTROL_INTERFACE_H_

#include <mav_msgs/Actuators.h>
#include <mavros_msgs/HilControls.h>
#include <ros/ros.h>

namespace rotors_hil {
// Default topic names
static const std::string kDefaultHilControlsSubTopic = "/mavros/hil_controls/hil_controls";
static const std::string kDefaultActuatorsPubTopic = "actuators";

// Default values
static constexpr bool kDefaultKeyTeleop = false;

class HilControlInterface {
 public:
  HilControlInterface();
  virtual ~HilControlInterface();

  // Callbacks
  void HilControlsCallback(const mavros_msgs::HilControlsConstPtr& hil_controls_msg);

 private:
  // ROS interface
  ros::NodeHandle nh_;
  ros::Subscriber hil_controls_sub_;
  ros::Publisher actuators_pub_;

  // Whether we are using a keyboard for teleop
  bool key_teleop_;
};
}

#endif // ROTORS_HIL_CONTROL_INTERFACE_H_
