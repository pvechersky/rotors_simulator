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
    : ModelPlugin(),
      node_handle_(0),
      ref_thrust_(0.0) {
}

GazeboFixedWingBasePlugin::~GazeboFixedWingBasePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
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
  node_handle_ = new ros::NodeHandle(namespace_);

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

  std::string actuators_topic;
  //std::string gui_throttle_topic;
  std::string reset_topic;
  getSdfParam<std::string>(_sdf, "actuatorsTopic", actuators_topic,
                           kDefaultActuatorsSubTopic);
  //getSdfParam<std::string>(_sdf, "guiThrottleTopic", gui_throttle_topic,
  //                         kDefaultGuiThrottleSubTopic);
  getSdfParam<std::string>(_sdf, "resetTopic", reset_topic,
                           kDefaultResetSubTopic);
  getSdfParam<double>(_sdf, "maxThrust", max_thrust_, kDefaultMaxThrust);

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboFixedWingBasePlugin::OnUpdate, this, _1));

  actuators_sub_ = node_handle_->subscribe(actuators_topic, 1, &GazeboFixedWingBasePlugin::actuatorsCallback, this);
  //gui_throttle_sub_ = node_handle_->subscribe(gui_throttle_topic, 1, &GazeboFixedWingBasePlugin::guiThrottleCallback, this);
  reset_sub_ = node_handle_->subscribe(reset_topic, 1, &GazeboFixedWingBasePlugin::resetCallback, this);
}

void GazeboFixedWingBasePlugin::OnUpdate(const common::UpdateInfo& _info) {
  link_->AddForce(math::Vector3(ref_thrust_, 0.0, 0.0));
}

void GazeboFixedWingBasePlugin::actuatorsCallback(const mav_msgs::ActuatorsConstPtr& act_msg) {
  ref_thrust_ = (act_msg->normalized.at(0) * 0.5 + 0.5) * max_thrust_;

  std::cout << "New thrust command: " << ref_thrust_ << std::endl;
  std::cout << "New aileron angle: " << act_msg->angles[0] << std::endl;
  std::cout << "New elevator angle: " << act_msg->angles[1] << std::endl;
  std::cout << "New rudder angle: " << act_msg->angles[2] << std::endl << std::endl;
}

/*void GazeboFixedWingBasePlugin::guiThrottleCallback(const std_msgs::UInt8ConstPtr& throttle_msg) {
  ref_thrust_ = (double)throttle_msg->data / 100.0 * max_thrust_;

  std::cout << "New thrust command: " << ref_thrust_ << std::endl;
}*/

void GazeboFixedWingBasePlugin::resetCallback(const std_msgs::BoolConstPtr& reset_msg) {
  if (reset_msg->data) {
    math::Pose pose = model_->GetWorldPose();
    pose.pos.x = 0.0;
    pose.pos.y = 0.0;
    pose.pos.z = 0.0;
    pose.rot.w = 1.0;
    pose.rot.x = 0.0;
    pose.rot.y = 0.0;
    pose.rot.z = 0.0;
    model_->SetWorldPose(pose);

    math::Vector3 lin_vel = model_->GetWorldLinearVel();
    lin_vel.x = 0.0;
    lin_vel.y = 0.0;
    lin_vel.z = 0.0;
    model_->SetLinearVel(lin_vel);

    math::Vector3 lin_acc = model_->GetWorldLinearAccel();
    lin_acc.x = 0.0;
    lin_acc.y = 0.0;
    lin_acc.z = 0.0;
    model_->SetLinearAccel(lin_acc);

    math::Vector3 ang_vel = model_->GetWorldAngularVel();
    ang_vel.x = 0.0;
    ang_vel.y = 0.0;
    ang_vel.z = 0.0;
    model_->SetAngularVel(ang_vel);

    math::Vector3 ang_acc = model_->GetWorldAngularAccel();
    ang_acc.x = 0.0;
    ang_acc.y = 0.0;
    ang_acc.z = 0.0;
    model_->SetAngularAccel(ang_acc);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFixedWingBasePlugin);
}
