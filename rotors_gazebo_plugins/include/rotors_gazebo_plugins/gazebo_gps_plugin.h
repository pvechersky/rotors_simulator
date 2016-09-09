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

#ifndef ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/TwistStamped.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <geometry_msgs/PoseStamped.h>

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/gps-conversions.h"

namespace gazebo {
// WGS84 constants
static constexpr double kEquatorialRadius = 6378137.0;
static constexpr double kPolarRadius = 6356752.3;
static constexpr double kFlattening = 1.0 / 298.257223563;
static constexpr double kEccentrity2 = 2 * kFlattening - kFlattening * kFlattening;

// Default reference values (Zurich: lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
static constexpr double kDefaultRefLat = 47.3667;
static constexpr double kDefaultRefLon = 8.5500;
static constexpr double kDefaultRefAlt = 500.0;
static constexpr double kDefaultRefHeading = 0.0;

// Default ground speed topic name
static const std::string kDefaultGroundSpeedPubTopic = "ground_speed";

class GazeboGpsPlugin : public ModelPlugin {
 public:
  GazeboGpsPlugin();
  virtual ~GazeboGpsPlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  std::string frame_id_;

  ros::NodeHandle* node_handle_;
  ros::Publisher gps_pub_;
  ros::Publisher ground_speed_pub_;

  ros::Publisher xyz_pub_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  double ref_lat_;
  double ref_lon_;
  double ref_alt_;
  double ref_heading_;
  double earth_radius_;
  double radius_north_;
  double radius_east_;

  sensor_msgs::NavSatFix gps_message_;
  geometry_msgs::TwistStamped ground_speed_msg_;

  geometry_msgs::PoseStamped xyz_msg_;

  double start_easting_;
  double start_northing_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H

/*#ifndef ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H

#include <random>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static const std::string kDefaultGroundSpeedPubTopic = "ground_speed";
static constexpr double kDefaultHorPosStdDev = 3.0;
static constexpr double kDefaultVerPosStdDev = 6.0;
static constexpr double kDefaultHorVelStdDev = 0.1;
static constexpr double kDefaultVerVelStdDev = 0.1;

class GazeboGpsPlugin : public SensorPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;

  GazeboGpsPlugin();
  virtual ~GazeboGpsPlugin();

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

  // Normal distributions for ground speed noise in x, y, and z directions
  NormalDistribution ground_speed_n_[3];

  // Random number generator
  std::random_device random_device_;
  std::mt19937 random_generator_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GPS_PLUGIN_H*/
