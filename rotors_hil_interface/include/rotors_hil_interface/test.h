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

#include <geometry_msgs/TwistStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/default_topics.h>
#include <mavros_msgs/HilControls.h>
#include <mavros_msgs/mavlink_convert.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>

namespace rotors_hil {
// Constants
static constexpr double kAirDensity = 1.18;
static constexpr double kGravityMagnitude = 9.8068;
static constexpr int kAllFieldsUpdated = 4095;

// Default values
static const std::string kDefaultAirSpeedSubTopic = "air_speed";
static const std::string kDefaultGroundSpeedSubTopic = "ground_speed";
static const std::string kDefaultHilControlsSubTopic = "/mavros/hil_controls/hil_controls";
static const std::string kDefaultPressureSubTopic = "air_pressure";
static const std::string kDefaultActuatorsPubTopic = "actuators";
static const std::string kDefaultMavlinkPubTopic = "/mavlink/to";
static constexpr bool kDefaultSensorLevelHil = true;
static constexpr double kDefaultHilGpsFrequency = 5.0;
static constexpr double kDefaultHilFrequency = 100.0;

class HilInterface {
 public:
  HilInterface();
  virtual ~HilInterface();

  void MainTaskSensorLevelHil();
  void MainTaskStateLevelHil();

  // Callbacks
  void AirSpeedCallback(const geometry_msgs::TwistStampedConstPtr &air_speed_msg);
  void GpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg);
  void GroundSpeedCallback(const geometry_msgs::TwistStampedConstPtr& ground_speed_msg);
  void HilControlsCallback(const mavros_msgs::HilControlsConstPtr& hil_controls_msg);
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void MagCallback(const sensor_msgs::MagneticFieldConstPtr& mag_msg);
  void PressureCallback(const sensor_msgs::FluidPressureConstPtr& pressure_msg);

  // Sensor data publishing
  void PublishHilGps();
  void PublishHilSensor();
  void PublishHilStateQtrn();

  // Whether we are running sensor-level or state-level HIL
  bool sensor_level_hil_;

 private:
  // ROS interface
  ros::NodeHandle nh_;
  ros::Subscriber air_speed_sub_;
  ros::Subscriber gps_sub_;
  ros::Subscriber ground_speed_sub_;
  ros::Subscriber hil_controls_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber mag_sub_;
  ros::Subscriber pressure_sub_;
  ros::Publisher actuators_pub_;
  ros::Publisher mavlink_pub_;
  ros::Rate rate_;

  // MAVLINK messages
  mavlink_hil_gps_t hil_gps_msg_;
  mavlink_hil_sensor_t hil_sensor_msg_;
  mavlink_hil_state_quaternion_t hil_state_qtrn_msg_;

  // GPS publishing interval
  u_int32_t hil_gps_interval_nsec_;

  // Publishing time trackers
  u_int32_t last_gps_pub_time_nsec_;
  u_int32_t last_imu_pub_time_nsec_;
  u_int32_t last_state_pub_time_nsec_;

  // Sensor data
  tf::Quaternion att_;          // Attitude quaternion
  float acc_x_;                 // X acceleration (m/s^2)
  float acc_y_;                 // Y acceleration (m/s^2)
  float acc_z_;                 // Z acceleration (m/s^2)
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
  int16_t vn_;                  // GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
  int16_t ve_;                  // GPS velocity in cm/s in EAST direction in earth-fixed NED frame
  int16_t vd_;                  // GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
  uint8_t fix_type_;            // < 0-1: no fix, 2: 2D fix, 3: 3D fix
  uint8_t satellites_visible_;  // Number of satellites visible. If unknown, set to 255
};
}

#endif // ROTORS_HIL_INTERFACE_H_
