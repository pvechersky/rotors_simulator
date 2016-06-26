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

namespace rotors_hil {

HilSensorsInterface::HilSensorsInterface():
    // TODO: add sensors or params or constants for these
  rate_(kDefaultHilImuFrequency),
  temperature_(15.0),
  eph_(100),
  epv_(100),
  cog_(0),
  ind_airspeed_(0),
  true_airspeed_(0),
  satellites_visible_(5)
{
  ros::NodeHandle pnh("~");

  double hil_gps_freq;
  double hil_imu_freq;
  double hil_state_freq;
  std::string air_speed_sub_topic;
  std::string gps_sub_topic;
  std::string ground_speed_sub_topic;
  std::string imu_sub_topic;
  std::string mag_sub_topic;
  std::string pressure_sub_topic;
  std::string mavlink_pub_topic;
  pnh.param("air_speed_topic", air_speed_sub_topic, kDefaultAirSpeedSubTopic);
  pnh.param("gps_topic", gps_sub_topic, std::string(mav_msgs::default_topics::GPS));
  pnh.param("ground_speed_topic", ground_speed_sub_topic, kDefaultGroundSpeedSubTopic);
  pnh.param("imu_topic", imu_sub_topic, std::string(mav_msgs::default_topics::IMU));
  pnh.param("mag_topic", mag_sub_topic, std::string(mav_msgs::default_topics::MAGNETIC_FIELD));
  pnh.param("pressure_topic", pressure_sub_topic, kDefaultPressureSubTopic);
  pnh.param("mavlink_pub_topic", mavlink_pub_topic, kDefaultMavlinkPubTopic);
  pnh.param("sensor_level_hil", sensor_level_hil_, kDefaultSensorLevelHil);
  pnh.param("hil_gps_frequency", hil_gps_freq, kDefaultHilGpsFrequency);
  pnh.param("hil_imu_frequency", hil_imu_freq, kDefaultHilImuFrequency);
  pnh.param("hil_state_frequency", hil_state_freq, kDefaultHilStateFrequency);

  air_speed_sub_ = nh_.subscribe(air_speed_sub_topic, 1, &HilSensorsInterface::AirSpeedCallback, this);
  gps_sub_ = nh_.subscribe(gps_sub_topic, 1, &HilSensorsInterface::GpsCallback, this);
  ground_speed_sub_ = nh_.subscribe(ground_speed_sub_topic, 1, &HilSensorsInterface::GroundSpeedCallback, this);
  imu_sub_ = nh_.subscribe(imu_sub_topic, 1, &HilSensorsInterface::ImuCallback, this);
  mag_sub_ = nh_.subscribe(mag_sub_topic, 1, &HilSensorsInterface::MagCallback, this);
  pressure_sub_ = nh_.subscribe(pressure_sub_topic, 1, &HilSensorsInterface::PressureCallback, this);

  mavlink_pub_ = nh_.advertise<mavros_msgs::Mavlink>(mavlink_pub_topic, 5);

  hil_gps_interval_nsec_ = 1.0 / hil_gps_freq * 1000000000;
  hil_imu_interval_nsec_ = 1.0 / hil_imu_freq * 1000000000;
  hil_state_interval_nsec_ = 1.0 / hil_state_freq * 1000000000;

  rate_ = (sensor_level_hil_) ? ros::Rate(hil_imu_freq) : ros::Rate(hil_state_freq);
}

HilSensorsInterface::~HilSensorsInterface() {
}

void HilSensorsInterface::MainTaskSensorLevelHil() {
  while (ros::ok()) {
    u_int32_t curr_nsec = ros::Time::now().nsec;

    if ((curr_nsec - last_gps_pub_time_nsec_) >= hil_gps_interval_nsec_) {
      last_gps_pub_time_nsec_ = curr_nsec;
      PublishHilGps();
    }

    if ((curr_nsec - last_imu_pub_time_nsec_) >= hil_imu_interval_nsec_) {
      last_imu_pub_time_nsec_ = curr_nsec;
      PublishHilSensor();
    }

    ros::spinOnce();
    rate_.sleep();
  }
}

void HilSensorsInterface::MainTaskStateLevelHil() {   
  while (ros::ok()) {
    u_int32_t curr_nsec = ros::Time::now().nsec;

    if ((curr_nsec - last_state_pub_time_nsec_) >= hil_state_interval_nsec_) {
      last_state_pub_time_nsec_ = curr_nsec;
      PublishHilStateQtrn();
    }

    ros::spinOnce();
    rate_.sleep();
  }
}

void HilSensorsInterface::AirSpeedCallback(const geometry_msgs::Vector3ConstPtr& air_speed_msg) {
  double air_speed = sqrt(air_speed_msg->x * air_speed_msg->x +
                          air_speed_msg->y * air_speed_msg->y +
                          air_speed_msg->z * air_speed_msg->z);

  // The same FOR NOW
  ind_airspeed_ = air_speed * 100.0;
  true_airspeed_ = air_speed * 100.0;
}

void HilSensorsInterface::GpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg) {
  lat_ = gps_msg->latitude * 10000000;
  lon_ = gps_msg->longitude * 10000000;
  alt_ = gps_msg->altitude * 1000;

  fix_type_ = (gps_msg->status.status > sensor_msgs::NavSatStatus::STATUS_NO_FIX) ? 3 : 0;

  // FOR NOW
  pressure_alt_ = gps_msg->altitude;
}

void HilSensorsInterface::GroundSpeedCallback(const geometry_msgs::Vector3ConstPtr& ground_speed_msg) {
  vn_ = ground_speed_msg->x * 100.0;
  ve_ = ground_speed_msg->y * 100.0;
  vd_ = ground_speed_msg->z * 100.0;

  vel_ = sqrt(vn_^2 + ve_^2 + vd_^2);
}

void HilSensorsInterface::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  acc_x_ = imu_msg->linear_acceleration.x;
  acc_y_ = -imu_msg->linear_acceleration.y;
  acc_z_ = -imu_msg->linear_acceleration.z;

  att_ = tf::Quaternion(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
  //att_ *= tf::Quaternion(0.0, 0.0, M_PI);

  gyro_x_ = imu_msg->angular_velocity.x;
  gyro_y_ = -imu_msg->angular_velocity.y;
  gyro_z_ = -imu_msg->angular_velocity.z;
}

void HilSensorsInterface::MagCallback(const sensor_msgs::MagneticFieldConstPtr &mag_msg) {
  // ROS magnetic field sensor message is in Tesla, while MAVLINK HIL_SENSOR message
  // measures magnetic field in Gauss. 1 Tesla = 10000 Gauss
  mag_x_ = mag_msg->magnetic_field.x * 10000;
  mag_y_ = mag_msg->magnetic_field.y * 10000;
  mag_z_ = mag_msg->magnetic_field.z * 10000;
}

void HilSensorsInterface::PressureCallback(const sensor_msgs::FluidPressureConstPtr &pressure_msg) {
  // ROS fluid pressure sensor message is in Pascals, while MAVLINK HIL_SENSOR message
  // measures fluid pressure in millibar. 1 Pascal = 0.01 millibar
  pressure_abs_ = pressure_msg->fluid_pressure * 0.01;

  // From the following formula: p_stag - p_static = 0.5 * rho * v^2
  pressure_diff_ = 0.5 * kAirDensity * ind_airspeed_ * ind_airspeed_ * 0.01 * 0.0001;
}

void HilSensorsInterface::PublishHilGps() {
  ros::Time current_time = ros::Time::now();

  hil_gps_msg_.time_usec = current_time.nsec * 0.001 + current_time.sec * 1000000;
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

  mavlink_message_t mmsg;
  mavlink_message_t* msg;
  mavlink_hil_gps_t* hil_gps_msg_ptr = &hil_gps_msg_;
  mavlink_msg_hil_gps_encode(1, 0, &mmsg, hil_gps_msg_ptr);
  msg = &mmsg;

  mavros_msgs::MavlinkPtr rmsg_hil_gps = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_gps->header.stamp.sec = current_time.sec;
  rmsg_hil_gps->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(*msg, *rmsg_hil_gps);

  mavlink_pub_.publish(rmsg_hil_gps);
}

void HilSensorsInterface::PublishHilSensor() {
  ros::Time current_time = ros::Time::now();

  hil_sensor_msg_.time_usec = current_time.nsec * 0.001 + current_time.sec * 1000000;
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

  mavlink_message_t mmsg;
  mavlink_message_t* msg;
  mavlink_hil_sensor_t* hil_sensor_msg_ptr = &hil_sensor_msg_;
  mavlink_msg_hil_sensor_encode(1, 0, &mmsg, hil_sensor_msg_ptr);
  msg = &mmsg;

  mavros_msgs::MavlinkPtr rmsg_hil_sensor = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_sensor->header.stamp.sec = current_time.sec;
  rmsg_hil_sensor->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(*msg, *rmsg_hil_sensor);

  mavlink_pub_.publish(rmsg_hil_sensor);
}

void HilSensorsInterface::PublishHilStateQtrn() {
  ros::Time current_time = ros::Time::now();

  hil_state_qtrn_msg_.time_usec = current_time.nsec * 0.001 + current_time.sec * 1000000;
  hil_state_qtrn_msg_.attitude_quaternion[0] = att_.w();
  hil_state_qtrn_msg_.attitude_quaternion[1] = att_.x();
  hil_state_qtrn_msg_.attitude_quaternion[2] = att_.y();
  hil_state_qtrn_msg_.attitude_quaternion[3] = att_.z();
  hil_state_qtrn_msg_.rollspeed = gyro_x_;
  hil_state_qtrn_msg_.pitchspeed = gyro_y_;
  hil_state_qtrn_msg_.yawspeed = gyro_z_;
  hil_state_qtrn_msg_.lat = lat_;
  hil_state_qtrn_msg_.lon = lon_;
  hil_state_qtrn_msg_.alt = alt_;
  hil_state_qtrn_msg_.vx = vn_;
  hil_state_qtrn_msg_.vy = ve_;
  hil_state_qtrn_msg_.vz = vd_;
  hil_state_qtrn_msg_.ind_airspeed = ind_airspeed_;
  hil_state_qtrn_msg_.true_airspeed = true_airspeed_;
  hil_state_qtrn_msg_.xacc = acc_x_ * 1000.0 / kGravityMagnitude;
  hil_state_qtrn_msg_.yacc = acc_y_ * 1000.0 / kGravityMagnitude;
  hil_state_qtrn_msg_.zacc = acc_z_ * 1000.0 / kGravityMagnitude;

  mavlink_message_t mmsg;
  mavlink_message_t* msg;
  mavlink_hil_state_quaternion_t* hil_state_qtrn_msg_ptr = &hil_state_qtrn_msg_;
  mavlink_msg_hil_state_quaternion_encode(1, 0, &mmsg, hil_state_qtrn_msg_ptr);
  msg = &mmsg;

  mavros_msgs::MavlinkPtr rmsg_hil_state_qtrn = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_state_qtrn->header.stamp.sec = current_time.sec;
  rmsg_hil_state_qtrn->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(*msg, *rmsg_hil_state_qtrn);

  mavlink_pub_.publish(rmsg_hil_state_qtrn);
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_hil_sensors_interface");
  rotors_hil::HilSensorsInterface hil_sensors_interface;

  if (hil_sensors_interface.sensor_level_hil_)
    hil_sensors_interface.MainTaskSensorLevelHil();
  else
    hil_sensors_interface.MainTaskStateLevelHil();

  return 0;
}
