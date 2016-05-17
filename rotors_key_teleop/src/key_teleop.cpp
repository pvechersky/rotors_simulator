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

#include "rotors_key_teleop/key_teleop.h"

namespace rotors_teleop {

KeyTeleop::KeyTeleop() {
  ConfigureTerminalInput();

  actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(kDefaultActuatorsPubTopic, 1);
}

KeyTeleop::~KeyTeleop() {
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
}

void KeyTeleop::KeyInputLoop() {
  char c;

  while (ros::ok()) {
    if (read(STDIN_FD, &c, 1) < 0) {
      ROS_ERROR("Error reading input from the terminal");
      Shutdown();
    }

    switch(c) {
      case KEYCODE_AILERON_LEFT:
        std::cout << "LEFT" << std::endl;
        break;
      case KEYCODE_AILERON_RIGHT:
        std::cout << "RIGHT" << std::endl;
        break;
      case KEYCODE_ELEVATOR_UP:
        std::cout << "UP" << std::endl;
        break;
      case KEYCODE_ELEVATOR_DOWN:
        std::cout << "DOWN" << std::endl;
        break;
      case KEYCODE_RUDDER_LEFT:
        break;
      case KEYCODE_RUDDER_RIGHT:
        break;
      case KEYCODE_THROTTLE_UP:
        throttle_ += 0.02;
        throttle_ = (throttle_ > 1.0) ? 1.0 : throttle_;
        break;
      case KEYCODE_THROTTLE_DOWN:
        throttle_ -= 0.02;
        throttle_ = (throttle_ < 0.0) ? 0.0 : throttle_;
        break;
      case KEYCODE_QUIT:
        Shutdown();
        break;
    }

    mav_msgs::Actuators act_msg;

    ros::Time current_time = ros::Time::now();

    act_msg.normalized.push_back(roll_ailerons_);
    act_msg.normalized.push_back(pitch_elevator_);
    act_msg.normalized.push_back(yaw_rudder_);
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
