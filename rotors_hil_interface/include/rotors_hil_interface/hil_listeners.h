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
#include <Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

namespace rotors_hil {
// Constants
static constexpr double kAirDensity = 1.18;
static constexpr double kGravityMagnitude = 9.8068;
static constexpr float kTemperature = 15.0;
static constexpr int kSatellitesVisible = 5;
static constexpr int kUnknown = 65535;

struct HilData {
  HilData() :
      temperature(kTemperature),
      eph(kUnknown),
      epv(kUnknown),
      cog(kUnknown),
      satellites_visible(kSatellitesVisible) {}

  Eigen::Quaterniond att;      // Attitude quaternion
  Eigen::Vector3f acc;         // Linear acceleration (m/s^2)
  Eigen::Vector3f gyro;        // Angular speed in body frame (rad / sec)
  Eigen::Vector3f mag;         // Magnetic field (Gauss)
  Eigen::Vector3i gps_vel;     // GPS velocity in cm/s in earth-fixed NED frame
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
  uint8_t fix_type;            // < 0-1: no fix, 2: 2D fix, 3: 3D fix
  uint8_t satellites_visible;  // Number of satellites visible. If unknown, set to 255
};

class HilListeners {
 public:
  HilListeners() {}
  virtual ~HilListeners() {}

  /// \brief Callback for handling Air Speed messages.
  /// \param[in] air_speed_msg An Air Speed message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void AirSpeedCallback(const geometry_msgs::TwistStampedConstPtr& air_speed_msg,
                        HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    Eigen::Vector3d air_velocity(air_speed_msg->twist.linear.x,
                                 air_speed_msg->twist.linear.y,
                                 air_speed_msg->twist.linear.z);

    double air_speed = air_velocity.norm();

    // TODO: The same FOR NOW

    // MAVLINK HIL_STATE_QUATERNION message measured airspeed in cm/s.
    hil_data->ind_airspeed = air_speed * 100.0;
    hil_data->true_airspeed = air_speed * 100.0;
  }

  /// \brief Callback for handling GPS messages.
  /// \param[in] gps_msg A GPS message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void GpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg,
                   HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    // MAVLINK HIL_GPS message measures latitude and longitude in degrees * 1e7
    // while altitude is reported in mm.
    hil_data->lat = gps_msg->latitude * 1e7;
    hil_data->lon = gps_msg->longitude * 1e7;
    hil_data->alt = gps_msg->altitude * 1000;

    hil_data->fix_type =
        (gps_msg->status.status > sensor_msgs::NavSatStatus::STATUS_NO_FIX) ? 3 : 0;

    // TODO: FOR NOW
    hil_data->pressure_alt = gps_msg->altitude;
  }

  /// \brief Callback for handling Ground Speed messages.
  /// \param[in] ground_speed_msg A ground speed message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void GroundSpeedCallback(const geometry_msgs::TwistStampedConstPtr &ground_speed_msg,
                           HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    hil_data->gps_vel = Eigen::Vector3i(ground_speed_msg->twist.linear.x,
                                        ground_speed_msg->twist.linear.y,
                                        ground_speed_msg->twist.linear.z) * 100.0;

    hil_data->vel = hil_data->gps_vel.norm();
  }

  /// \brief Callback for handling IMU messages.
  /// \param[in] imu_msg An IMU message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg,
                   HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    hil_data->acc = Eigen::Vector3f(imu_msg->linear_acceleration.x,
                                    imu_msg->linear_acceleration.y,
                                    imu_msg->linear_acceleration.z);

    hil_data->att = Eigen::Quaterniond(imu_msg->orientation.w,
                                       imu_msg->orientation.x,
                                       imu_msg->orientation.y,
                                       imu_msg->orientation.z);

    hil_data->gyro = Eigen::Vector3f(imu_msg->angular_velocity.x,
                                     imu_msg->angular_velocity.y,
                                     imu_msg->angular_velocity.z);
  }

  /// \brief Callback for handling Magnetometer messages.
  /// \param[in] mag_msg A Magnetometer message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void MagCallback(const sensor_msgs::MagneticFieldConstPtr &mag_msg,
                   HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    // ROS magnetic field sensor message is in Tesla, while
    // MAVLINK HIL_SENSOR message measures magnetic field in Gauss.
    // 1 Tesla = 10000 Gauss
    hil_data->mag = Eigen::Vector3f(mag_msg->magnetic_field.x,
                                    mag_msg->magnetic_field.y,
                                    mag_msg->magnetic_field.z) * 10000.0;
  }

  /// \brief Callback for handling Air Pressure messages.
  /// \param[in] pressure_msg An Air Pressure message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void PressureCallback(const sensor_msgs::FluidPressureConstPtr &pressure_msg,
                        HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    // ROS fluid pressure sensor message is in Pascals, while
    // MAVLINK HIL_SENSOR message measures fluid pressure in millibar.
    // 1 Pascal = 0.01 millibar
    hil_data->pressure_abs = pressure_msg->fluid_pressure * 0.01;

    // From the following formula: p_stag - p_static = 0.5 * rho * v^2
    // HIL air speed is in cm/s and is converted to m/s for the purpose of
    // computing pressure.
    hil_data->pressure_diff = 0.5 * kAirDensity * hil_data->ind_airspeed *
            hil_data->ind_airspeed * 0.01 * 0.0001;
  }

 private:
  /// Mutex lock for thread safety of writing hil data.
  boost::mutex mtx_;
};

}

#endif // ROTORS_HIL_LISTENERS_H_
