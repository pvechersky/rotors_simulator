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

#include "rotors_mavlink_interface/mavlink_interface.h"
//#include "rotors_simulator/rotors_mavlink_interface/include/rotors_mavlink_interface/mavlink_interface.h"

namespace rotors_mavlink {

MavlinkInterface::MavlinkInterface() {
  ros::NodeHandle pnh("~");

  std::string gps_sub_topic;
  std::string imu_sub_topic;
  std::string hil_sensor_mavlink_pub_topic;
  pnh.param("gps_topic", gps_sub_topic, kDefaultGpsSubTopic);
  pnh.param("imu_topic", imu_sub_topic, kDefaultImuSubTopic);
  pnh.param("mavlink_pub_topic", hil_sensor_mavlink_pub_topic, kDefaultMavlinkHilSensorPubTopic);

  gps_sub_ = nh_.subscribe(gps_sub_topic, 10, &MavlinkInterface::GpsCallback, this);
  imu_sub_ = nh_.subscribe(kDefaultImuSubTopic, 10, &MavlinkInterface::ImuCallback, this);
  hil_sensor_pub_ = nh_.advertise<mavros_msgs::Mavlink>(kDefaultMavlinkHilSensorPubTopic, 10);
}

MavlinkInterface::~MavlinkInterface() {
}

void MavlinkInterface::GpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg) {

}

void MavlinkInterface::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  mavlink_message_t mmsg;

  //math::Pose T_W_I = model_->GetWorldPose();
  //math::Vector3 pos_W_I = T_W_I.pos;  // Use the models'world position for GPS and pressure alt.

  //math::Quaternion C_W_I;
  //C_W_I.w = imu_message->orientation.w;
  //C_W_I.x = imu_message->orientation.x;
  //C_W_I.y = imu_message->orientation.y;
  //C_W_I.z = imu_message->orientation.z;

  //math::Vector3 mag_I = C_W_I.RotateVectorReverse(mag_W_); // TODO: Add noise based on bais and variance like for imu and gyro

  hil_sensor_msg_.time_usec = imu_msg->header.stamp.nsec*1000;
  hil_sensor_msg_.xacc = imu_msg->linear_acceleration.x;
  hil_sensor_msg_.yacc = imu_msg->linear_acceleration.y;
  hil_sensor_msg_.zacc = imu_msg->linear_acceleration.z;
  hil_sensor_msg_.xgyro = imu_msg->angular_velocity.x;
  hil_sensor_msg_.ygyro = imu_msg->angular_velocity.y;
  hil_sensor_msg_.zgyro = imu_msg->angular_velocity.z;
  //hil_sensor_msg_.xmag = mag_I.x;
  //hil_sensor_msg_.ymag = mag_I.y;
  //hil_sensor_msg_.zmag = mag_I.z;
  hil_sensor_msg_.abs_pressure = 0.0;
  hil_sensor_msg_.diff_pressure = 0.0;
  //hil_sensor_msg_.pressure_alt = pos_W_I.z;
  hil_sensor_msg_.temperature = 0.0;
  hil_sensor_msg_.fields_updated = 4095;  // 0b1111111111111 (All updated since new data with new noise added always)

  mavlink_hil_sensor_t* hil_msg = &hil_sensor_msg_;
  mavlink_msg_hil_sensor_encode(1, 240, &mmsg, hil_msg);
  mavlink_message_t* msg = &mmsg;

  mavros_msgs::MavlinkPtr rmsg = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg->header.stamp = ros::Time::now();
  mavros_msgs::mavlink::convert(*msg, *rmsg);

  hil_sensor_pub_.publish(rmsg);
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_mavlink_interface");
  rotors_mavlink::MavlinkInterface mavlink_interface;

  ros::spin();

  return 0;
}
