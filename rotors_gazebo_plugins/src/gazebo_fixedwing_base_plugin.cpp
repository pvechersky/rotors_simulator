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

#include "rotors_gazebo_plugins/gazebo_fixedwing_base_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboFixedWingBasePlugin::GazeboFixedWingBasePlugin()
    : ModelPlugin() {
}

GazeboFixedWingBasePlugin::~GazeboFixedWingBasePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboFixedWingBasePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model and the world
  model_ = _model;
  world_ = model_->GetWorld();

  // Get the robot namespace
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_fixedwing_base_plugin] Please specify a robotNamespace.\n";

  // Get the link name
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_fixedwing_base_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_fixedwing_base_plugin] Couldn't find specified link \"" << link_name << "\".");

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboFixedWingBasePlugin::OnUpdate, this, _1));
}

void GazeboFixedWingBasePlugin::OnUpdate(const common::UpdateInfo& _info) {
  //if (!initialized_)
  //{

  float vel_x = link_->GetWorldLinearVel().x;

  //if (vel_x < 40.0)
  //{
  //  link_->AddForce(math::Vector3(150.0, 0.0, 0.0));
  //}

  //std::cout << "Vel x: " << vel_x << std::endl;
  //std::cout << "Pos x: " << link_->GetWorldPose().pos.x << std::endl;
  //std::cout << "Pos z: " << link_->GetWorldPose().pos.z << std::endl;

    //initialized_ = true;
  //}

  //double altitude = model_->GetWorldPose().pos.z;
  //std::cout << altitude << std::endl;

  //common::Time current_time  = world_->GetSimTime();

  // Get the current pose from Gazebo
  //math::Pose pose = link_->GetWorldPose();

  //pose.pos.x += 0.001;

  //link_->SetWorldPose(pose);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFixedWingBasePlugin);
}
