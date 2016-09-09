#ifndef ROTORS_GAZEBO_PLUGINS_GPS_TEMP_H
#define ROTORS_GAZEBO_PLUGINS_GPS_TEMP_H

#include <random>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/PoseStamped.h>

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/gps-conversions.h"

namespace gazebo {
// Default values
static const std::string kDefaultGroundSpeedPubTopic = "ground_speed";
static constexpr double kDefaultHorPosStdDev = 3.0;
static constexpr double kDefaultVerPosStdDev = 6.0;
static constexpr double kDefaultHorVelStdDev = 0.1;
static constexpr double kDefaultVerVelStdDev = 0.1;

class GpsTemp : public SensorPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;

  GpsTemp();
  virtual ~GpsTemp();

 protected:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  void OnUpdate();

 private:
  // ROS interface
  ros::NodeHandle* node_handle_;
  ros::Publisher gps_pub_;
  ros::Publisher ground_speed_pub_;
  std::string gps_topic_;
  std::string ground_speed_topic_;

  ros::Publisher xyz_pub_;

  // Pointer to the parent sensor
  sensors::GpsSensorPtr parent_sensor_;

  // Pointer to the world
  physics::WorldPtr world_;

  // Pointer to the sensor link
  physics::LinkPtr link_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  // GPS message to be published on sensor update
  sensor_msgs::NavSatFix gps_message_;

  // Ground speed message to be publised on sensor update
  geometry_msgs::TwistStamped ground_speed_message_;

  geometry_msgs::PoseStamped xyz_msg_;

  // Normal distributions for ground speed noise in x, y, and z directions
  NormalDistribution ground_speed_n_[3];

  // Random number generator
  std::random_device random_device_;
  std::mt19937 random_generator_;

  double start_easting_;
  double start_northing_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GPS_TEMP_H
