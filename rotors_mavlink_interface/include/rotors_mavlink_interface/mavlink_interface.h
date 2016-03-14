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


#ifndef ROTORS_MAVLINK_INTERFACE_H_
#define ROTORS_MAVLINK_INTERFACE_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

//#include "mavros/mavros_msgs/include/mavros_msgs/mavlink_convert.h"

#include <mavros_msgs/mavlink_convert.h>

namespace rotors_mavlink {

static const std::string kDefaultGpsSubTopic = "gps";
static const std::string kDefaultImuSubTopic = "imu";
static const std::string kDefaultMavlinkHilSensorPubTopic = "mavlink/from";

class MavlinkInterface {
 public:
  MavlinkInterface();
  virtual ~MavlinkInterface();

  void GpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg);
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

 private:
  ros::NodeHandle nh_;
  ros::Subscriber gps_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher hil_sensor_pub_;

  mavlink_hil_sensor_t hil_sensor_msg_;
};
}

#endif // ROTORS_MAVLINK_INTERFACE_H_
