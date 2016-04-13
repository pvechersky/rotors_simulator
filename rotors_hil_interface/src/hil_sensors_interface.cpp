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

#include "rotors_hil_interface/hil_sensors_interface.h"
//#include "rotors_simulator/rotors_hil_interface/include/rotors_hil_interface/hil_sensors_interface.h"

namespace rotors_hil {

HilSensorsInterface::HilSensorsInterface():
  received_gps_(false),
  received_imu_(false),
  received_mag_(false),
  received_pressure_(false),
  pressure_diff_(0.0),
  pressure_alt_(0.0),
  temperature_(15.0),
  eph_(65535),
  epv_(65535),
  vel_(65535),
  cog_(65535),
  fix_type_(0),
  vn_(0),
  ve_(0),
  vd_(0),
  satellites_visible_(255)
{
  ros::NodeHandle pnh("~");

  std::string gps_sub_topic;
  std::string imu_sub_topic;
  std::string mag_sub_topic;
  std::string pressure_sub_topic;
  std::string set_mode_sub_topic;
  std::string mavlink_pub_topic;
  pnh.param("gps_topic", gps_sub_topic, std::string(mav_msgs::default_topics::GPS));
  pnh.param("imu_topic", imu_sub_topic, std::string(mav_msgs::default_topics::IMU));
  pnh.param("mag_topic", mag_sub_topic, std::string(mav_msgs::default_topics::MAGNETIC_FIELD));
  pnh.param("pressure_topic", pressure_sub_topic, kDefaultPressureSubTopic);
  pnh.param("set_mode_topic", set_mode_sub_topic, kDefaultSetModeSubTopic);
  pnh.param("mavlink_pub_topic", mavlink_pub_topic, kDefaultMavlinkPubTopic);

  gps_sub_ = nh_.subscribe(gps_sub_topic, 1, &HilSensorsInterface::GpsCallback, this);
  imu_sub_ = nh_.subscribe(imu_sub_topic, 1, &HilSensorsInterface::ImuCallback, this);
  mag_sub_ = nh_.subscribe(mag_sub_topic, 1, &HilSensorsInterface::MagCallback, this);
  pressure_sub_ = nh_.subscribe(pressure_sub_topic, 1, &HilSensorsInterface::PressureCallback, this);
  set_mode_sub_ = nh_.subscribe(set_mode_sub_topic, 1, &HilSensorsInterface::SetModeCallback, this);

  mavlink_pub_ = nh_.advertise<mavros_msgs::Mavlink>(mavlink_pub_topic, 5);
}

HilSensorsInterface::~HilSensorsInterface() {
}

void HilSensorsInterface::MainTask() {
  ros::Rate rate_outer(2.0);
  ros::Rate rate_inner(20.0);

  // The outer loop sends the sensor data to autopilot at lower frequency
  while (ros::ok()) {
    // Wait until we have received updates from all the sensor
    while (!AreAllSensorsUpdated()) {
      ros::spinOnce();
      rate_inner.sleep();
    }

    SendHilSensorData();

    ros::spinOnce();
    rate_outer.sleep();
  }
}

void HilSensorsInterface::GpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg) {
  lat_ = gps_msg->latitude * 10000000;
  lon_ = gps_msg->longitude * 10000000;
  alt_ = gps_msg->altitude * 1000;

  fix_type_ = (gps_msg->status.status > sensor_msgs::NavSatStatus::STATUS_NO_FIX) ? 3 : 0;

  if (!received_gps_)
    received_gps_ = true;
}

void HilSensorsInterface::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  acc_x_ = imu_msg->linear_acceleration.x;
  acc_y_ = imu_msg->linear_acceleration.y;
  acc_z_ = imu_msg->linear_acceleration.z;

  gyro_x_ = imu_msg->angular_velocity.x;
  gyro_y_ = imu_msg->angular_velocity.y;
  gyro_z_ = imu_msg->angular_velocity.z;

  if (!received_imu_)
    received_imu_ = true;
}

void HilSensorsInterface::MagCallback(const sensor_msgs::MagneticFieldConstPtr &mag_msg) {
  // ROS magnetic field sensor message is in Tesla, while MAVLINK hil sensor message
  // measures magnetic field in Gauss. 1 Tesla = 10000 Gauss
  mag_x_ = mag_msg->magnetic_field.x * 10000;
  mag_y_ = mag_msg->magnetic_field.y * 10000;
  mag_z_ = mag_msg->magnetic_field.z * 10000;

  if (!received_mag_)
    received_mag_ = true;
}

void HilSensorsInterface::PressureCallback(const sensor_msgs::FluidPressureConstPtr &pressure_msg) {
  // ROS fluid pressure sensor message is in Pascals, while MAVLINK hil sensor message
  // measures fluid pressure in millibar. 1 Pascal = 0.01 millibar
  pressure_abs_ = pressure_msg->fluid_pressure * 0.01;

  if (!received_pressure_)
    received_pressure_ = true;
}

void HilSensorsInterface::SetModeCallback(const std_msgs::UInt8ConstPtr& set_mode_msg) {
  mavlink_message_t mmsg;

  cmd_msg_.target_system = 1;
  cmd_msg_.target_component = 0;
  cmd_msg_.command = MAV_CMD_DO_SET_MODE;
  cmd_msg_.confirmation = 1;
  cmd_msg_.param1 = set_mode_msg->data;
  cmd_msg_.param2 = 0;

  mavlink_command_long_t* cmd_msg_ptr = &cmd_msg_;
  mavlink_msg_command_long_encode(1, 0, &mmsg, cmd_msg_ptr);
  mavlink_message_t* msg = &mmsg;

  ros::Time current_time = ros::Time::now();

  mavros_msgs::MavlinkPtr rmsg = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg->header.stamp.sec = current_time.sec;
  rmsg->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(*msg, *rmsg);

  mavlink_pub_.publish(rmsg);
}

void HilSensorsInterface::SendHilSensorData() {
  mavlink_message_t mmsg;
  mavlink_message_t* msg;

  ros::Time current_time = ros::Time::now();

  // Encode and publish the hil_sensor message
  /*hil_sensor_msg_.time_usec = current_time.nsec * 1000;
  hil_sensor_msg_.xacc = 0.0f;
  hil_sensor_msg_.yacc = 0.0f;
  hil_sensor_msg_.zacc = 9.81f;
  hil_sensor_msg_.xgyro = 0.0f;
  hil_sensor_msg_.ygyro = 0.0f;
  hil_sensor_msg_.zgyro = 0.0f;
  hil_sensor_msg_.xmag = 0.21475f;
  hil_sensor_msg_.ymag = 0.00797f;
  hil_sensor_msg_.zmag = 0.42817f;
  hil_sensor_msg_.abs_pressure = 0.0;
  hil_sensor_msg_.diff_pressure = 0.0;
  hil_sensor_msg_.pressure_alt = 500.0f;
  hil_sensor_msg_.temperature = 15.0f;
  hil_sensor_msg_.fields_updated = 4095;*/
  hil_sensor_msg_.time_usec = current_time.nsec*1000;
  hil_sensor_msg_.xacc = acc_x_;
  hil_sensor_msg_.yacc = acc_y_;
  hil_sensor_msg_.zacc = acc_z_;
  hil_sensor_msg_.xgyro = gyro_x_;
  hil_sensor_msg_.ygyro = gyro_y_;
  hil_sensor_msg_.zgyro = gyro_z_;
  hil_sensor_msg_.xmag = mag_x_;
  hil_sensor_msg_.ymag = mag_y_;
  hil_sensor_msg_.zmag = mag_z_;
  hil_sensor_msg_.abs_pressure = pressure_abs_;
  hil_sensor_msg_.diff_pressure = pressure_diff_;
  hil_sensor_msg_.pressure_alt = pressure_alt_;
  hil_sensor_msg_.temperature = temperature_;
  hil_sensor_msg_.fields_updated = kAllFieldsUpdated;

  mavlink_hil_sensor_t* hil_sensor_msg_ptr = &hil_sensor_msg_;
  mavlink_msg_hil_sensor_encode(1, 0, &mmsg, hil_sensor_msg_ptr);
  msg = &mmsg;

  mavros_msgs::MavlinkPtr rmsg_hil_sensor = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_sensor->header.stamp.sec = current_time.sec;
  rmsg_hil_sensor->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(*msg, *rmsg_hil_sensor);

  mavlink_pub_.publish(rmsg_hil_sensor);

  // Encode and publish the hil_gps message
  /*hil_gps_msg_.time_usec = current_time.nsec * 1000;
  hil_gps_msg_.fix_type = 3;
  hil_gps_msg_.lat = 47.3667 * 10000000;
  hil_gps_msg_.lon = 8.5500 * 10000000;
  hil_gps_msg_.alt = 500 * 1000;
  hil_gps_msg_.eph = 65535;
  hil_gps_msg_.epv = 65535;
  hil_gps_msg_.vel = 0;
  hil_gps_msg_.vn = 0;
  hil_gps_msg_.ve = 0;
  hil_gps_msg_.vd = 0;
  hil_gps_msg_.cog = 0;
  hil_gps_msg_.satellites_visible = 5;*/
  hil_gps_msg_.time_usec = current_time.nsec*1000;
  hil_gps_msg_.fix_type = fix_type_;
  hil_gps_msg_.lat = lat_;
  hil_gps_msg_.lon = lon_;
  hil_gps_msg_.alt = alt_;
  hil_gps_msg_.eph = eph_;
  hil_gps_msg_.epv = epv_;
  hil_gps_msg_.vel = vel_;
  hil_gps_msg_.vn = vn_;
  hil_gps_msg_.ve = ve_;
  hil_gps_msg_.vd = vd_;
  hil_gps_msg_.cog = cog_;
  hil_gps_msg_.satellites_visible = satellites_visible_;

  mavlink_hil_gps_t* hil_gps_msg_ptr = &hil_gps_msg_;
  mavlink_msg_hil_gps_encode(1, 0, &mmsg, hil_gps_msg_ptr);
  msg = &mmsg;

  mavros_msgs::MavlinkPtr rmsg_hil_gps = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_gps->header.stamp.sec = current_time.sec;
  rmsg_hil_gps->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(*msg, *rmsg_hil_gps);

  mavlink_pub_.publish(rmsg_hil_gps);

  // Encode and publish the hil_state_quaternion message
  /*hil_state_qtrn_msg_.time_usec = current_time.nsec * 1000;
  hil_state_qtrn_msg_.attitude_quaternion[0] = 1.0;
  hil_state_qtrn_msg_.attitude_quaternion[1] = 0.0;
  hil_state_qtrn_msg_.attitude_quaternion[2] = 0.0;
  hil_state_qtrn_msg_.attitude_quaternion[3] = 0.0;
  hil_state_qtrn_msg_.rollspeed = 0.0;
  hil_state_qtrn_msg_.pitchspeed = 0.0;
  hil_state_qtrn_msg_.yawspeed = 0.0;
  hil_state_qtrn_msg_.lat = 47.3667 * 10000000;
  hil_state_qtrn_msg_.lon = 8.5500 * 10000000;
  hil_state_qtrn_msg_.alt = 500 * 1000;
  hil_state_qtrn_msg_.vx = 0;
  hil_state_qtrn_msg_.vy = 0;
  hil_state_qtrn_msg_.vz = 0;
  hil_state_qtrn_msg_.ind_airspeed = 0;
  hil_state_qtrn_msg_.true_airspeed = 0;
  hil_state_qtrn_msg_.xacc = 0;
  hil_state_qtrn_msg_.yacc = 0;
  hil_state_qtrn_msg_.zacc = 9.81f;*/
  hil_state_qtrn_msg_.time_usec = current_time.nsec * 1000;
  hil_state_qtrn_msg_.attitude_quaternion[0] = 1.0;
  hil_state_qtrn_msg_.attitude_quaternion[1] = 0.0;
  hil_state_qtrn_msg_.attitude_quaternion[2] = 0.0;
  hil_state_qtrn_msg_.attitude_quaternion[3] = 0.0;
  hil_state_qtrn_msg_.rollspeed = gyro_x_;
  hil_state_qtrn_msg_.pitchspeed = gyro_y_;
  hil_state_qtrn_msg_.yawspeed = gyro_z_;
  hil_state_qtrn_msg_.lat = lat_;
  hil_state_qtrn_msg_.lon = lon_;
  hil_state_qtrn_msg_.alt = alt_;
  hil_state_qtrn_msg_.vx = 0;
  hil_state_qtrn_msg_.vy = 0;
  hil_state_qtrn_msg_.vz = 0;
  hil_state_qtrn_msg_.ind_airspeed = 0;
  hil_state_qtrn_msg_.true_airspeed = 0;
  hil_state_qtrn_msg_.xacc = acc_x_;
  hil_state_qtrn_msg_.yacc = acc_y_;
  hil_state_qtrn_msg_.zacc = acc_z_;

  mavlink_hil_state_quaternion_t* hil_state_qtrn_msg_ptr = &hil_state_qtrn_msg_;
  mavlink_msg_hil_state_quaternion_encode(1, 0, &mmsg, hil_state_qtrn_msg_ptr);
  msg = &mmsg;

  mavros_msgs::MavlinkPtr rmsg_hil_state_qtrn = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_state_qtrn->header.stamp.sec = current_time.sec;
  rmsg_hil_state_qtrn->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(*msg, *rmsg_hil_state_qtrn);

  mavlink_pub_.publish(rmsg_hil_state_qtrn);

  ClearAllSensorsUpdateStatuses();
}

void HilSensorsInterface::ClearAllSensorsUpdateStatuses() {
  received_gps_ = false;
  received_imu_ = false;
  received_mag_ = false;
  received_pressure_ = false;
}

bool HilSensorsInterface::AreAllSensorsUpdated() {
  return (received_gps_ && received_imu_ && received_mag_ && received_pressure_);
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_hil_sensors_interface");
  rotors_hil::HilSensorsInterface hil_sensors_interface;

  hil_sensors_interface.MainTask();

  return 0;
}
