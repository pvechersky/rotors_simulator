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


#ifndef ROTORS_PING_MAV_H_
#define ROTORS_PING_MAV_H_

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <mavros_msgs/mavlink_convert.h>

namespace rotors_mavlink {
// Default values
static const std::string kDefaultMavlinkSubTopic = "mavlink/from";
static const std::string kDefaultMavlinkPubTopic = "mavlink/to";

class PingMav {
 public:
  PingMav();
  virtual ~PingMav();

  // Mavlink callback
  void MavlinkCallback(const mavros_msgs::MavlinkConstPtr& mavros_msg);

  // Ping functions
  void PingAll();

 private:
  // ROS interface
  ros::NodeHandle nh_;
  ros::Subscriber mavlink_sub_;
  ros::Publisher mavlink_pub_;

  // Message handle
  mavlink_ping_t ping_msg_;
};
}

#endif // ROTORS_PING_MAV_H_
