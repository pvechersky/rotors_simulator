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

#include "rotors_mavlink_interface/ping_mav.h"
//#include "rotors_simulator/rotors_mavlink_interface/include/rotors_mavlink_interface/ping_mav.h"

namespace rotors_mavlink {

PingMav::PingMav() {
  ros::NodeHandle pnh("~");

  std::string mavlink_sub_topic;
  std::string mavlink_pub_topic;
  pnh.param("mavlink_sub_topic", mavlink_sub_topic, kDefaultMavlinkSubTopic);
  pnh.param("mavlink_pub_topic", mavlink_pub_topic, kDefaultMavlinkPubTopic);

  mavlink_sub_ = nh_.subscribe(mavlink_sub_topic, 10, &PingMav::MavlinkCallback, this);
  mavlink_pub_ = nh_.advertise<mavros_msgs::Mavlink>(mavlink_pub_topic, 10);
}

PingMav::~PingMav() {
}

void PingMav::MavlinkCallback(const mavros_msgs::MavlinkConstPtr& mavros_msg) {
  if (mavros_msg->msgid == MAVLINK_MSG_ID_PING)
  {
    mavlink_message_t* mavlink_msg;
    mavros_msgs::mavlink::convert(*mavros_msg, *mavlink_msg);

    mavlink_ping_t ping;
    mavlink_msg_ping_decode(mavlink_msg, &ping);

    std::cout << "Received a ping" << std::endl;
    std::cout << std::to_string(ping.target_system) << std::endl;
    std::cout << std::to_string(ping.target_component) << std::endl;
  }
}

void PingMav::PingAll() {
  ros::Time current_time = ros::Time::now();

  mavlink_message_t mmsg;

  ping_msg_.time_usec = current_time.nsec  * 0.001;
  ping_msg_.seq = 1;
  ping_msg_.target_system = 0;
  ping_msg_.target_component = 0;

  mavlink_ping_t* ping_msg_ptr = &ping_msg_;
  mavlink_msg_ping_encode(0, 0, &mmsg, ping_msg_ptr);
  mavlink_message_t* msg = &mmsg;

  mavros_msgs::MavlinkPtr rmsg = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg->header.stamp.sec = current_time.sec;
  rmsg->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(*msg, *rmsg);

  mavlink_pub_.publish(rmsg);
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_ping_mav");
  rotors_mavlink::PingMav ping_mav;

  ros::Rate rate(1.0);

  while (ros::ok())
  {
    ping_mav.PingAll();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
