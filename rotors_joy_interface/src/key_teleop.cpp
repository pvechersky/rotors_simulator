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

#include "rotors_joy_interface/key_teleop.h"

namespace rotors_teleop {

KeyTeleop::KeyTeleop() {
  ConfigureTerminalInput();

  ros::NodeHandle pnh("~");
  std::string actuators_pub_topic;
  pnh.param("actuators_pub_topic", actuators_pub_topic, kDefaultActuatorsPubTopic);
  pnh.param("control_timeout", control_timeout_, kDefaultControlTimeout);
  pnh.param("controls_sensitivity", sensitivity_, kDefaultSensitivity);

  actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(actuators_pub_topic, 1);
}

KeyTeleop::~KeyTeleop() {
  Shutdown();
}

void KeyTeleop::ConfigureTerminalInput() {
  // Get the console attributes
  termios attributes_new;
  tcgetattr(STDIN_FD, &attributes_old_);

  //std::cout << attributes_old_.c_cflag << std::endl;
  memcpy(&attributes_new, &attributes_old_, sizeof(termios));

  // Disable canonical mode and echoing of characters
  attributes_new.c_lflag &= ~(ICANON | ECHO);

  // Minimum number of characters
  attributes_new.c_cc[VMIN] = 1;
  attributes_new.c_cc[VEOL] = 1;
  attributes_new.c_cc[VEOF] = 2;

  // Set the new attributes
  tcsetattr(STDIN_FD, TCSANOW, &attributes_new);

  ROS_INFO("Terminal configured for keyboard teleop");
}

void KeyTeleop::KeyInputLoop() {
  char c;

  while (ros::ok()) {
    if (read(STDIN_FD, &c, 1) < 0) {
      ROS_ERROR("Error reading input from the terminal");
      Shutdown();
    }

    double curr_time = ros::Time::now().toSec();

    switch(c) {
      case KEYCODE_ROLL_LEFT:
        roll_ailerons_ -= sensitivity_;
        roll_ailerons_ = (roll_ailerons_ < -1.0) ? -1.0 : roll_ailerons_;
        last_aileron_time_ = curr_time;
        break;
      case KEYCODE_ROLL_RIGHT:
        roll_ailerons_ += sensitivity_;
        roll_ailerons_ = (roll_ailerons_ > 1.0) ? 1.0 : roll_ailerons_;
        last_aileron_time_ = curr_time;
        break;
      case KEYCODE_PITCH_UP:
        pitch_elevator_ -= sensitivity_;
        pitch_elevator_ = (pitch_elevator_ < -1.0) ? -1.0 : pitch_elevator_;
        last_elevator_time_ = curr_time;
        break;
      case KEYCODE_PITCH_DOWN:
        pitch_elevator_ += sensitivity_;
        pitch_elevator_ = (pitch_elevator_ > 1.0) ? 1.0 : pitch_elevator_;
        last_elevator_time_ = curr_time;
        break;
      case KEYCODE_YAW_LEFT:
        yaw_rudder_ -= sensitivity_;
        yaw_rudder_ = (yaw_rudder_ < -1.0) ? -1.0 : yaw_rudder_;
        last_rudder_time_ = curr_time;
        break;
      case KEYCODE_YAW_RIGHT:
        yaw_rudder_ += sensitivity_;
        yaw_rudder_ = (yaw_rudder_ > 1.0) ? 1.0 : yaw_rudder_;
        last_rudder_time_ = curr_time;
        break;
      case KEYCODE_THROTTLE_UP:
        throttle_ += sensitivity_;
        throttle_ = (throttle_ > 1.0) ? 1.0 : throttle_;
        break;
      case KEYCODE_THROTTLE_DOWN:
        throttle_ -= sensitivity_;
        throttle_ = (throttle_ < 0.0) ? 0.0 : throttle_;
        break;
      case KEYCODE_RESET:
        ROS_INFO("Keyboard teleop - resetting controls");
        roll_ailerons_ = 0.0;
        pitch_elevator_ = 0.0;
        yaw_rudder_ = 0.0;
        throttle_ = 0.0;
      case KEYCODE_QUIT:
        ROS_INFO("Keyboard teleop - shuttind down");
        Shutdown();
        break;
      default:
        break;
    }

    // If it has been too long since we received certain commands we set those controls to zero
    roll_ailerons_ = ((curr_time - last_aileron_time_) > control_timeout_) ? 0.0 : roll_ailerons_;
    pitch_elevator_ = ((curr_time - last_elevator_time_) > control_timeout_) ? 0.0 : pitch_elevator_;
    yaw_rudder_ = ((curr_time - last_rudder_time_) > control_timeout_) ? 0.0 : yaw_rudder_;

    mav_msgs::Actuators act_msg;

    ros::Time current_time = ros::Time::now();

    act_msg.normalized.push_back(roll_ailerons_);
    act_msg.normalized.push_back(pitch_elevator_);
    act_msg.normalized.push_back(yaw_rudder_);
    act_msg.normalized.push_back(0.0);
    act_msg.normalized.push_back(roll_ailerons_);
    act_msg.normalized.push_back(throttle_);

    act_msg.header.stamp.sec = current_time.sec;
    act_msg.header.stamp.nsec = current_time.nsec;

    actuators_pub_.publish(act_msg);

    ros::spinOnce();
  }
}

void KeyTeleop::Shutdown() {
  // Restore the old attributes
  tcsetattr(STDIN_FD, TCSANOW, &attributes_old_);
  ros::shutdown();
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_key_teleop");

  rotors_teleop::KeyTeleop key_teleop;
  key_teleop.KeyInputLoop();

  return 0;
}
