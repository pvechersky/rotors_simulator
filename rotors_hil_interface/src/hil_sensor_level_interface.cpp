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

HilSensorLevelInterface::HilSensorLevelInterface() {
  /*ros::NodeHandle pnh("~");

  std::string hil_controls_sub_topic;
  std::string actuators_pub_topic;
  pnh.param("hil_controls_sub_topic", hil_controls_sub_topic, kDefaultHilControlsSubTopic);
  pnh.param("actuators_pub_topic", actuators_pub_topic, kDefaultActuatorsPubTopic);*/

  air_speed_sub_ = nh_.subscribe<geometry_msgs::Vector3>("test", 1, boost::bind(&HilListeners::AirSpeedCallback, &hil_listeners_, _1, &hil_data_));
  gps_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("test", 1, boost::bind(&HilListeners::GpsCallback, &hil_listeners_, _1, &hil_data_));
  ground_speed_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("test", 1, boost::bind(&HilListeners::GroundSpeedCallback, &hil_listeners_, _1, &hil_data_));
  imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("test", 1, boost::bind(&HilListeners::ImuCallback, &hil_listeners_, _1, &hil_data_));
  mag_sub_ = nh_.subscribe<sensor_msgs::MagneticField>("test", 1, boost::bind(&HilListeners::MagnetometerCallback, &hil_listeners_, _1, &hil_data_));
  pressure_sub_ = nh_.subscribe<sensor_msgs::FluidPressure>("test", 1, boost::bind(&HilListeners::PressureCallback, &hil_listeners_, _1, &hil_data_));
}

HilSensorLevelInterface::~HilSensorLevelInterface() {
}

std::vector<mavros_msgs::Mavlink> HilSensorLevelInterface::CollectData() {
  ros::Time current_time = ros::Time::now();

  mavlink_message_t mmsg;
  std::vector<mavros_msgs::Mavlink> hil_msgs;

  hil_gps_msg_.time_usec = current_time.nsec * 0.001 + current_time.sec * 1000000;
  hil_gps_msg_.fix_type = hil_data_.fix_type;
  hil_gps_msg_.lat = hil_data_.lat;
  hil_gps_msg_.lon = hil_data_.lon;
  hil_gps_msg_.alt = hil_data_.alt;
  hil_gps_msg_.eph = hil_data_.eph;
  hil_gps_msg_.epv = hil_data_.epv;
  hil_gps_msg_.vel = hil_data_.vel;
  hil_gps_msg_.vn = hil_data_.vn;
  hil_gps_msg_.ve = hil_data_.ve;
  hil_gps_msg_.vd = hil_data_.vd;
  hil_gps_msg_.cog = hil_data_.cog;
  hil_gps_msg_.satellites_visible = hil_data_.satellites_visible;

  mavlink_hil_gps_t* hil_gps_msg_ptr = &hil_gps_msg_;
  mavlink_msg_hil_gps_encode(1, 0, &mmsg, hil_gps_msg_ptr);

  mavros_msgs::MavlinkPtr rmsg_hil_gps = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_gps->header.stamp.sec = current_time.sec;
  rmsg_hil_gps->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(mmsg, *rmsg_hil_gps);

  hil_msgs.push_back(*rmsg_hil_gps);

  hil_sensor_msg_.time_usec = current_time.nsec * 0.001 + current_time.sec * 1000000;
  hil_sensor_msg_.xacc = hil_data_.acc_x;
  hil_sensor_msg_.yacc = hil_data_.acc_y;
  hil_sensor_msg_.zacc = hil_data_.acc_z;
  hil_sensor_msg_.xgyro = hil_data_.gyro_x;
  hil_sensor_msg_.ygyro = hil_data_.gyro_y;
  hil_sensor_msg_.zgyro = hil_data_.gyro_z;
  hil_sensor_msg_.xmag = hil_data_.mag_x;
  hil_sensor_msg_.ymag = hil_data_.mag_y;
  hil_sensor_msg_.zmag = hil_data_.mag_z;
  hil_sensor_msg_.abs_pressure = hil_data_.pressure_abs;
  hil_sensor_msg_.diff_pressure = hil_data_.pressure_diff;
  hil_sensor_msg_.pressure_alt = hil_data_.pressure_alt;
  hil_sensor_msg_.temperature = hil_data_.temperature;
  hil_sensor_msg_.fields_updated = kAllFieldsUpdated;

  mavlink_hil_sensor_t* hil_sensor_msg_ptr = &hil_sensor_msg_;
  mavlink_msg_hil_sensor_encode(1, 0, &mmsg, hil_sensor_msg_ptr);

  mavros_msgs::MavlinkPtr rmsg_hil_sensor = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_sensor->header.stamp.sec = current_time.sec;
  rmsg_hil_sensor->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(mmsg, *rmsg_hil_sensor);

  hil_msgs.push_back(*rmsg_hil_sensor);

  return hil_msgs;
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
