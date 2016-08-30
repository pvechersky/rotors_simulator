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

#ifndef ROTORS_HIL_INTERFACE_H_
#define ROTORS_HIL_INTERFACE_H_

#include <mav_msgs/Actuators.h>
#include <mav_msgs/default_topics.h>
#include <mavros_msgs/HilControls.h>
#include <mavros_msgs/mavlink_convert.h>

#include <rotors_hil_interface/hil_listeners.h>

namespace rotors_hil {
// Constants
static constexpr int kAllFieldsUpdated = 4095;

// Default topic names
static const std::string kDefaultHilControlsSubTopic = "/mavros/hil_controls/hil_controls";
static const std::string kDefaultActuatorsPubTopic = "actuators";

class HilInterface {
 public:
  std::vector<mavros_msgs::Mavlink> virtual CollectData() = 0;

 protected:
  // ROS interface
  ros::NodeHandle nh_;

  HilData hil_data_;

  HilListeners hil_listeners_;
};

class HilSensorLevelInterface : public HilInterface {
 public:
  HilSensorLevelInterface();
  virtual ~HilSensorLevelInterface();

  std::vector<mavros_msgs::Mavlink> CollectData();

 private:
  // ROS interfaces
  ros::Subscriber air_speed_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber ground_speed_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber pressure_sub_;

  // MAVLINK messages
  mavlink_hil_gps_t hil_gps_msg_;
  mavlink_hil_sensor_t hil_sensor_msg_;
};

class HilStateLevelInterface : public HilInterface {
 public:
  HilStateLevelInterface();
  virtual ~HilStateLevelInterface();

  std::vector<mavros_msgs::Mavlink> CollectData();

 private:
  // MAVLINK messages
  mavlink_hil_state_quaternion_t hil_state_qtrn_msg_;
};
}

#endif // ROTORS_HIL_INTERFACE_H_
