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

#ifndef ROTORS_KEY_TELEOP_H_
#define ROTORS_KEY_TELEOP_H_

#include <ros/ros.h>
#include <termios.h>

#include <mav_msgs/Actuators.h>

namespace rotors_teleop {
// File descriptor constant
static constexpr int STDIN_FD = 0;

// Key assignment constants
static constexpr int KEYCODE_THROTTLE_UP = 0x77;    // 'w'
static constexpr int KEYCODE_THROTTLE_DOWN = 0x73;  // 's'
static constexpr int KEYCODE_YAW_LEFT = 0x61;       // 'a'
static constexpr int KEYCODE_YAW_RIGHT = 0x64;      // 'd'
static constexpr int KEYCODE_PITCH_UP = 0x42;       // up arrow
static constexpr int KEYCODE_PITCH_DOWN = 0x41;     // down arrow
static constexpr int KEYCODE_ROLL_LEFT = 0x44;      // left arrow
static constexpr int KEYCODE_ROLL_RIGHT = 0x43;     // right arrow
static constexpr int KEYCODE_RESET = 0x52;          // 'R'
static constexpr int KEYCODE_QUIT = 0x51;           // 'Q'

// Default values
static const std::string kDefaultActuatorsPubTopic = "gazebo/command/motor_speed";
static constexpr double kDefaultControlTimeout = 0.5;
static constexpr double kDefaultSensitivity = 0.05;

class KeyTeleop {
 public:
  KeyTeleop();
  virtual ~KeyTeleop();

  void ConfigureTerminalInput();
  void KeyInputLoop();
  void Shutdown();

 private:
  // ROS interface
  ros::NodeHandle nh_;
  ros::Publisher actuators_pub_;

  // Control values
  double roll_ailerons_;
  double pitch_elevator_;
  double yaw_rudder_;
  double throttle_;

  // Controls sensitivity
  double sensitivity_;

  // Control timers
  double last_aileron_time_;
  double last_elevator_time_;
  double last_rudder_time_;
  double control_timeout_;

  // Object to store original input terminal attributes
  termios attributes_old_;
};
}

#endif // ROTORS_KEY_TELEOP_H_
