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

#include "rotors_hil_interface/hil_control_interface.h"
//#include "rotors_simulator/rotors_hil_interface/include/rotors_hil_interface/hil_control_interface.h"

namespace rotors_hil {

HilControlInterface::HilControlInterface():
  base_mode_(0),
  system_status_(255)
{
  ros::NodeHandle pnh("~");

  std::string mavlink_sub_topic;
  std::string mav_mode_pub_topic;
  std::string mav_status_pub_topic;
  pnh.param("mavlink_sub_topic", mavlink_sub_topic, kDefaultMavlinkSubTopic);
  pnh.param("mav_mode_pub_topic", mav_mode_pub_topic, kDefaultMavModePubTopic);
  pnh.param("mav_status_pub_topic", mav_status_pub_topic, kDefaultMavStatusPubTopic);

  mavlink_sub_ = nh_.subscribe(mavlink_sub_topic, 10, &HilControlInterface::MavlinkCallback, this);

  mav_mode_pub_ = nh_.advertise<std_msgs::UInt8>(mav_mode_pub_topic, 1);
  mav_status_pub_ = nh_.advertise<std_msgs::UInt8>(mav_status_pub_topic, 1);
}

HilControlInterface::~HilControlInterface() {
}

void HilControlInterface::MavlinkCallback(const mavros_msgs::MavlinkConstPtr& mavros_msg) {
  if (mavros_msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    mavlink_message_t* mavlink_msg;
    mavros_msgs::mavlink::convert(*mavros_msg, *mavlink_msg);

    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(mavlink_msg, &heartbeat);

    if (base_mode_ != heartbeat.base_mode) {
      base_mode_ = heartbeat.base_mode;

      std_msgs::UInt8 mav_mode_msg;
      mav_mode_msg.data = base_mode_;
      mav_mode_pub_.publish(mav_mode_msg);
    }

    if (system_status_ != heartbeat.system_status) {
      system_status_ = heartbeat.system_status;

      std_msgs::UInt8 mav_status_msg;
      mav_status_msg.data = system_status_;
      mav_status_pub_.publish(mav_status_msg);
    }
  }
  else if (mavros_msg->msgid == MAVLINK_MSG_ID_HIL_CONTROLS) {

  }
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_hil_control_interface");
  rotors_hil::HilControlInterface hil_control_interface;

  ros::spin();

  return 0;
}
