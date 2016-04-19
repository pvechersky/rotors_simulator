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

#include "rotors_gazebo_plugins/gazebo_servo_plugin.h"

#include <boost/bind.hpp>

namespace gazebo {

GazeboServoPlugin::GazeboServoPlugin()
    : ModelPlugin(),
      node_handle_(0),
      ref_angle_(0.0) {}

GazeboServoPlugin::~GazeboServoPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboServoPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_servo_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_servo_plugin] Please specify a jointName.\n";
  // Get the pointer to the link
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_servo_plugin] Couldn't find specified joint \"" << joint_name_ << "\".");

  std::string command_sub_topic;
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic, kDefaultCommandSubTopic);
  getSdfParam<double>(_sdf, "velocityGain", velocity_gain_, kDefaultVelocityGain);
  getSdfParam<double>(_sdf, "minAngle", min_angle_, kDefaultMinAngle);
  getSdfParam<double>(_sdf, "maxAngle", max_angle_, kDefaultMaxAngle);
  getSdfParam<int>(_sdf, "channel", channel_, kDefaultChannel);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboServoPlugin::OnUpdate, this, _1));

  command_sub_ = node_handle_->subscribe(command_sub_topic, 1, &GazeboServoPlugin::AngleCallback, this);
}

void GazeboServoPlugin::AngleCallback(const mav_msgs::ActuatorsConstPtr& servo_angles) {
  ROS_ASSERT_MSG(servo_angles->angles.size() > channel_,
                 "You tried to access index %d of the Angles message array which is of size %d.",
                 channel_, servo_angles->angles.size());
  ref_angle_ = (max_angle_ + min_angle_) * 0.5 + (max_angle_ - min_angle_) * 0.5 * servo_angles->angles.at(channel_);
}

// This gets called by the world update start event.
void GazeboServoPlugin::OnUpdate(const common::UpdateInfo& _info) {
  double current_angle = joint_->GetAngle(0).Radian();
  double err = ref_angle_ - current_angle;
  joint_->SetVelocity(0, err * velocity_gain_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboServoPlugin);
}
