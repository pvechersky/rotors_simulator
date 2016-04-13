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

#include <ros/ros.h>
#include <std_msgs/UInt8.h>

#include <mav_msgs/Actuators.h>
#include <mavros_msgs/mavlink_convert.h>

namespace rotors_hil {
// Default topic names
static const std::string kDefaultMavlinkSubTopic = "/mavlink/from";
static const std::string kDefaultMavModePubTopic = "mav_mode";
static const std::string kDefaultMavStatusPubTopic = "mav_status";
static const std::string kDefaultActuatorsPubTopic = "actuators";

// Default actuator values [rad]
static constexpr double kDefaultRollMin = -M_PI * 0.25;
static constexpr double kDefaultRollMax = M_PI * 0.25;
static constexpr double kDefaultPitchMin = -M_PI * 0.1;
static constexpr double kDefaultPitchMax = M_PI * 0.1;
static constexpr double kDefaultYawMin = -M_PI * 0.25;
static constexpr double kDefaultYawMax = M_PI * 0.25;

class HilControlInterface {
 public:
  HilControlInterface();
  virtual ~HilControlInterface();

  // Callbacks
  void MavlinkCallback(const mavros_msgs::MavlinkConstPtr& mavros_msg);

 private:
  // ROS interface
  ros::NodeHandle nh_;
  ros::Subscriber mavlink_sub_;
  ros::Publisher mav_mode_pub_;
  ros::Publisher mav_status_pub_;
  ros::Publisher actuators_pub_;

  // MAV diagnostics
  uint8_t base_mode_;
  uint8_t system_status_;

  // Actuator constraints
  double roll_min_;
  double roll_max_;
  double pitch_min_;
  double pitch_max_;
  double yaw_min_;
  double yaw_max_;
};
}

#endif // ROTORS_HIL_CONTROL_INTERFACE_H_
