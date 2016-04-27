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

#ifndef ROTORS_HIL_SENSORS_INTERFACE_H_
#define ROTORS_HIL_SENSORS_INTERFACE_H_

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

#include <mav_msgs/default_topics.h>
#include <mavros_msgs/mavlink_convert.h>

namespace rotors_hil {
// Constants
static constexpr int kAllFieldsUpdated = 4095;
static constexpr double kDefaultGravityMagnitude = 9.8068;

// Default values
static const std::string kDefaultAirSpeedSubTopic = "air_speed";
static const std::string kDefaultGroundSpeedSubTopic = "ground_speed";
static const std::string kDefaultPressureSubTopic = "air_pressure";
static const std::string kDefaultSetModeSubTopic = "set_mode";
static const std::string kDefaultMavlinkPubTopic = "/mavlink/to";

class HilSensorsInterface {
 public:
  HilSensorsInterface();
  virtual ~HilSensorsInterface();

  void MainTask();

  // Callbacks
  void AirSpeedCallback(const geometry_msgs::Vector3ConstPtr& air_speed_msg);
  void GpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg);
  void GroundSpeedCallback(const geometry_msgs::Vector3ConstPtr& ground_speed_msg);
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void MagCallback(const sensor_msgs::MagneticFieldConstPtr& mag_msg);
  void PressureCallback(const sensor_msgs::FluidPressureConstPtr& pressure_msg);
  void SetModeCallback(const std_msgs::UInt8ConstPtr& set_mode_msg);

  // Sensor data management
  void SendHilSensorData();
  void ClearAllSensorsUpdateStatuses();
  bool AreAllSensorsUpdated();

 private:
  // ROS interface
  ros::NodeHandle nh_;
  ros::Subscriber air_speed_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber ground_speed_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber pressure_sub_;
  ros::Subscriber set_mode_sub_;
  ros::Publisher mavlink_pub_;

  // MAVLINK messages
  mavlink_hil_gps_t hil_gps_msg_;
  mavlink_hil_sensor_t hil_sensor_msg_;
  mavlink_hil_state_quaternion_t hil_state_qtrn_msg_;
  mavlink_command_long_t cmd_msg_;

  // Sensor update trackers
  bool received_air_speed_;
  bool received_gps_;
  bool received_ground_speed_;
  bool received_imu_;
  bool received_mag_;
  bool received_pressure_;

  // Sensor data
  float att_w_;                 // w component of attitude quaternion
  float att_x_;                 // x component of attitude quaternion
  float att_y_;                 // y component of attitude quaternion
  float att_z_;                 // z component of attitude quaternion
  float gyro_x_;                // Angular speed around X axis in body frame (rad / sec)
  float gyro_y_;                // Angular speed around Y axis in body frame (rad / sec)
  float gyro_z_;                // Angular speed around Z axis in body frame (rad / sec)
  float mag_x_;                 // X Magnetic field (Gauss)
  float mag_y_;                 // Y Magnetic field (Gauss)
  float mag_z_;                 // Z Magnetic field (Gauss)
  float pressure_abs_;          // Absolute pressure in millibar
  float pressure_diff_;         // Differential pressure (airspeed) in millibar
  float pressure_alt_;          // Altitude calculated from pressure
  float temperature_;           // Temperature in degrees celsius
  uint32_t lat_;                // Latitude (WGS84), in degrees * 1E7
  uint32_t lon_;                // Longitude (WGS84), in degrees * 1E7
  uint32_t alt_;                // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
  uint16_t eph_;                // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
  uint16_t epv_;                // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
  uint16_t vel_;                // GPS ground speed (m/s * 100). If unknown, set to: 65535
  uint16_t cog_;                // Course over ground (NOT heading, but direction of movement) in degrees * 100. If unknown, set to: 65535
  uint16_t ind_airspeed_;       // Indicated airspeed, expressed as m/s * 100
  uint16_t true_airspeed_;      // True airspeed, expressed as m/s * 100*/
  int16_t acc_x_;               // X acceleration (milli g's)
  int16_t acc_y_;               // Y acceleration (milli g's)
  int16_t acc_z_;               // Z acceleration (milli g's)
  int16_t vn_;                  // GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
  int16_t ve_;                  // GPS velocity in cm/s in EAST direction in earth-fixed NED frame
  int16_t vd_;                  // GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
  uint8_t fix_type_;            // < 0-1: no fix, 2: 2D fix, 3: 3D fix
  uint8_t satellites_visible_;  // Number of satellites visible. If unknown, set to 255
};
}

#endif // ROTORS_HIL_SENSORS_INTERFACE_H_
