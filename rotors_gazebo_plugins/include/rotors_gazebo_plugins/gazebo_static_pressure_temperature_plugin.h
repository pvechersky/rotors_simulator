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

#ifndef ROTORS_GAZEBO_PLUGINS_STATIC_PRESSURE_TEMPERATURE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_STATIC_PRESSURE_TEMPERATURE_PLUGIN_H

#include <random>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Constants
static constexpr double kRs = 8314.32; /* Nm/ (kmol K), gas constant */
static constexpr double kM0 = 28.9644; /* kg/kmol, mean molecular weight of air */
static constexpr double kG0 = 9.80665; /* m/s^2, acceleration due to gravity at 45.5425 deg lat */
static constexpr double kR0 = 6356766.0; /* m, Earth radius at g0 */
static constexpr double kP0 = 101325.0; /* Pa, air pressure at g0 */
static constexpr double kT0 = 288.15; /* K, standard sea-level temperature */
static constexpr double kTl = 0.0065; /* K/m, temperature lapse */
static constexpr double kAs = kG0 * kM0 / (kRs * -kTl);

// Default values
static const std::string kDefaultPressurePubTopic = "air_pressure";
static const std::string kDefaultTemperaturePubTopic = "temperature";
static constexpr double kDefaultPressureVar = 0.0; /* Pa^2, pressure variance */
static constexpr double kDefaultTemperatureVar = 0.0; /* (Degrees Celcius)^2, temperature variance */

class GazeboStaticPressureTemperaturePlugin : public ModelPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;

  GazeboStaticPressureTemperaturePlugin();
  virtual ~GazeboStaticPressureTemperaturePlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  ros::NodeHandle* node_handle_;
  ros::Publisher pressure_pub_;
  ros::Publisher temperature_pub_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  NormalDistribution pressure_n_;
  NormalDistribution temperature_n_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

  double ref_alt_;

  sensor_msgs::FluidPressure pressure_message_;
  sensor_msgs::Temperature temperature_message_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_STATIC_PRESSURE_TEMPERATURE_PLUGIN_H
