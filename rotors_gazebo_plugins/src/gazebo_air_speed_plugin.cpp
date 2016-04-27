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

#include "rotors_gazebo_plugins/gazebo_air_speed_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboAirSpeedPlugin::GazeboAirSpeedPlugin()
    : ModelPlugin(),
      node_handle_(0),
      ground_speed_(math::Vector3(0.0, 0.0, 0.0)),
      wind_speed_(math::Vector3(0.0, 0.0, 0.0)) {
}

GazeboAirSpeedPlugin::~GazeboAirSpeedPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboAirSpeedPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model and the world
  model_ = _model;
  world_ = model_->GetWorld();

  // Use the robot namespace to create the node handle
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_air_speed_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  std::string air_speed_topic_;
  std::string ground_speed_topic;
  std::string wind_speed_topic;
  getSdfParam<std::string>(_sdf, "airSpeedTopic", air_speed_topic_, kDefaultAirSpeedPubTopic);
  getSdfParam<std::string>(_sdf, "groundSpeedTopic", ground_speed_topic, kDefaultGroundSpeedSubTopic);
  getSdfParam<std::string>(_sdf, "windSpeedTopic", wind_speed_topic, kDefaultWindSpeedSubTopic);

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboAirSpeedPlugin::OnUpdate, this, _1));

  air_speed_pub_ = node_handle_->advertise<geometry_msgs::Vector3>(air_speed_topic_, 1);

  ground_speed_sub_ = node_handle_->subscribe(ground_speed_topic, 1, &GazeboAirSpeedPlugin::GroundSpeedCallback, this);
  wind_speed_sub_ = node_handle_->subscribe(wind_speed_topic, 1, &GazeboAirSpeedPlugin::WindSpeedCallback, this);
}

void GazeboAirSpeedPlugin::OnUpdate(const common::UpdateInfo& _info) {
  math::Vector3 air_speed = wind_speed_ - ground_speed_;

  air_speed_msg_.x = air_speed.x;
  air_speed_msg_.y = air_speed.y;
  air_speed_msg_.z = air_speed.z;

  air_speed_pub_.publish(air_speed_msg_);
}

void GazeboAirSpeedPlugin::GroundSpeedCallback(const geometry_msgs::Vector3ConstPtr& ground_speed_msg) {
  ground_speed_.x = ground_speed_msg->x;
  ground_speed_.y = ground_speed_msg->y;
  ground_speed_.z = ground_speed_msg->z;
}

void GazeboAirSpeedPlugin::WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed_msg) {
  wind_speed_.x = wind_speed_msg->velocity.x;
  wind_speed_.y = wind_speed_msg->velocity.y;
  wind_speed_.z = wind_speed_msg->velocity.z;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboAirSpeedPlugin);
}
