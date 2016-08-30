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

#ifndef ROTORS_HIL_LISTENERS_H_
#define ROTORS_HIL_LISTENERS_H_

#include <boost/thread/mutex.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

namespace rotors_hil {
// Constants
static constexpr double kAirDensity = 1.18;
static constexpr double kGravityMagnitude = 9.8068;

struct HilData {
  //tf::Quaternion att;          // Attitude quaternion
  float acc_x;                 // X acceleration (m/s^2)
  float acc_y;                 // Y acceleration (m/s^2)
  float acc_z;                 // Z acceleration (m/s^2)
  float gyro_x;                // Angular speed around X axis in body frame (rad / sec)
  float gyro_y;                // Angular speed around Y axis in body frame (rad / sec)
  float gyro_z;                // Angular speed around Z axis in body frame (rad / sec)
  float mag_x;                 // X Magnetic field (Gauss)
  float mag_y;                 // Y Magnetic field (Gauss)
  float mag_z;                 // Z Magnetic field (Gauss)
  float pressure_abs;          // Absolute pressure in millibar
  float pressure_diff;         // Differential pressure (airspeed) in millibar
  float pressure_alt;          // Altitude calculated from pressure
  float temperature;           // Temperature in degrees celsius
  uint32_t lat;                // Latitude (WGS84), in degrees * 1E7
  uint32_t lon;                // Longitude (WGS84), in degrees * 1E7
  uint32_t alt;                // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
  uint16_t eph;                // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
  uint16_t epv;                // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
  uint16_t vel;                // GPS ground speed (m/s * 100). If unknown, set to: 65535
  uint16_t cog;                // Course over ground (NOT heading, but direction of movement) in degrees * 100. If unknown, set to: 65535
  uint16_t ind_airspeed;       // Indicated airspeed, expressed as m/s * 100
  uint16_t true_airspeed;      // True airspeed, expressed as m/s * 100*/
  int16_t vn;                  // GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
  int16_t ve;                  // GPS velocity in cm/s in EAST direction in earth-fixed NED frame
  int16_t vd;                  // GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
  uint8_t fix_type;            // < 0-1: no fix, 2: 2D fix, 3: 3D fix
  uint8_t satellites_visible;  // Number of satellites visible. If unknown, set to 255
};

class HilListeners {
 public:
  HilListeners();
  virtual ~HilListeners() {}

  void AirSpeedCallback(const geometry_msgs::Vector3ConstPtr& air_speed_msg,
                        HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    double air_speed = sqrt(air_speed_msg->x * air_speed_msg->x +
                            air_speed_msg->y * air_speed_msg->y +
                            air_speed_msg->z * air_speed_msg->z);

    // The same FOR NOW
    hil_data->ind_airspeed = air_speed * 100.0;
    hil_data->true_airspeed = air_speed * 100.0;
  }

  void GpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg,
                   HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    hil_data->lat = gps_msg->latitude * 10000000;
    hil_data->lon = gps_msg->longitude * 10000000;
    hil_data->alt = gps_msg->altitude * 1000;

    hil_data->fix_type = (gps_msg->status.status > sensor_msgs::NavSatStatus::STATUS_NO_FIX) ? 3 : 0;

    // FOR NOW
    hil_data->pressure_alt = gps_msg->altitude;
  }

  void GroundSpeedCallback(const geometry_msgs::TwistStampedConstPtr &ground_speed_msg,
                           HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    hil_data->vn = ground_speed_msg->twist.linear.x * 100.0;
    hil_data->ve = -ground_speed_msg->twist.linear.y * 100.0;
    hil_data->vd = -ground_speed_msg->twist.linear.z * 100.0;

    hil_data->vel = sqrt((hil_data->vn)^2 + (hil_data->ve)^2 + (hil_data->vd)^2);
  }

  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg,
                   HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    hil_data->acc_x = imu_msg->linear_acceleration.x;
    hil_data->acc_y = -imu_msg->linear_acceleration.y;
    hil_data->acc_z = -imu_msg->linear_acceleration.z;

    //att_ = tf::Quaternion(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
    //att_ *= tf::Quaternion(0.0, 0.0, M_PI);

    hil_data->gyro_x = imu_msg->angular_velocity.x;
    hil_data->gyro_y = -imu_msg->angular_velocity.y;
    hil_data->gyro_z = -imu_msg->angular_velocity.z;
  }

  void MagnetometerCallback(const sensor_msgs::MagneticFieldConstPtr &mag_msg,
                            HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    // ROS magnetic field sensor message is in Tesla, while MAVLINK HIL_SENSOR message
    // measures magnetic field in Gauss. 1 Tesla = 10000 Gauss
    hil_data->mag_x = mag_msg->magnetic_field.x * 10000;
    hil_data->mag_y = -mag_msg->magnetic_field.y * 10000;
    hil_data->mag_z = -mag_msg->magnetic_field.z * 10000;
  }

  void PressureCallback(const sensor_msgs::FluidPressureConstPtr &pressure_msg,
                        HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    // ROS fluid pressure sensor message is in Pascals, while MAVLINK HIL_SENSOR message
    // measures fluid pressure in millibar. 1 Pascal = 0.01 millibar
    hil_data->pressure_abs = pressure_msg->fluid_pressure * 0.01;

    // From the following formula: p_stag - p_static = 0.5 * rho * v^2
    hil_data->pressure_diff = 0.5 * kAirDensity * hil_data->ind_airspeed * hil_data->ind_airspeed * 0.01 * 0.0001;
  }

 private:
  /// \brief Mutex lock for thread safety of writing hil data
  boost::mutex mtx_;
};

}

#endif // ROTORS_HIL_LISTENERS_H_
