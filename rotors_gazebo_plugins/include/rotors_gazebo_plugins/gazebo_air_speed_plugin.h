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

#ifndef ROTORS_GAZEBO_PLUGINS_AIR_SPEED_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_AIR_SPEED_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

#include "rotors_comm/WindSpeed.h"
#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static const std::string kDefaultAirSpeedPubTopic = "air_speed";
static const std::string kDefaultGroundSpeedSubTopic = "ground_speed";
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";
static constexpr double kDefaultAirSpeedVariance = 0.1;

class GazeboAirSpeedPlugin : public ModelPlugin {
 typedef std::normal_distribution<> NormalDistribution;

 public:
  GazeboAirSpeedPlugin();
  virtual ~GazeboAirSpeedPlugin();

  //void GroundSpeedCallback(const geometry_msgs::Vector3ConstPtr& ground_speed_msg);
  void WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed_msg);

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  //void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;

  ros::NodeHandle* node_handle_;
  ros::Publisher air_speed_pub_;
  //ros::Subscriber ground_speed_sub_;
  ros::Subscriber wind_speed_sub_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

  NormalDistribution air_speed_n_;

  //math::Vector3 ground_speed_;
  //math::Vector3 wind_speed_;

  geometry_msgs::Vector3 air_speed_msg_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_AIR_SPEED_PLUGIN_H
