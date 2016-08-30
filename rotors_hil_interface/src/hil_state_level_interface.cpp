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

#include "rotors_hil_interface/hil_interface.h"

namespace rotors_hil {

HilStateLevelInterface::HilStateLevelInterface() {
  /*ros::NodeHandle pnh("~");

  std::string hil_controls_sub_topic;
  std::string actuators_pub_topic;
  pnh.param("hil_controls_sub_topic", hil_controls_sub_topic, kDefaultHilControlsSubTopic);
  pnh.param("actuators_pub_topic", actuators_pub_topic, kDefaultActuatorsPubTopic);

  hil_controls_sub_ = nh_.subscribe(hil_controls_sub_topic, 1,
                                      &HilInterface::HilControlsCallback);

  actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(actuators_pub_topic, 1);*/
}

HilStateLevelInterface::~HilStateLevelInterface() {
}

std::vector<mavros_msgs::Mavlink> HilStateLevelInterface::CollectData() {

}

/*void HilSensorLevelInterface::HilControlsCallback(const mavros_msgs::HilControlsConstPtr& hil_controls_msg) {
  mav_msgs::Actuators act_msg;

  ros::Time current_time = ros::Time::now();

  act_msg.normalized.push_back(hil_controls_msg->roll_ailerons);
  act_msg.normalized.push_back(hil_controls_msg->pitch_elevator);
  act_msg.normalized.push_back(hil_controls_msg->yaw_rudder);

  act_msg.normalized.push_back(hil_controls_msg->throttle);

  act_msg.header.stamp.sec = current_time.sec;
  act_msg.header.stamp.nsec = current_time.nsec;

  actuators_pub_.publish(act_msg);
}*/
}
