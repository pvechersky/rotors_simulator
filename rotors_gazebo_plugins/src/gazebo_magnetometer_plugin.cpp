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

#include "rotors_gazebo_plugins/gazebo_magnetometer_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboMagnetometerPlugin::GazeboMagnetometerPlugin()
    : ModelPlugin(),
      node_handle_(0),
      random_generator_(random_device_()) {
}

GazeboMagnetometerPlugin::~GazeboMagnetometerPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboMagnetometerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model and the world
  model_ = _model;
  world_ = model_->GetWorld();

  // Use the robot namespace to create the node handle
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  // Use the link name as the frame id
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_magnetometer_plugin] Couldn't find specified link \"" << link_name << "\".");

  frame_id_ = link_name;

  double ref_mag_north;
  double ref_mag_east;
  double ref_mag_down;
  sdf::Vector3 noise_normal;
  sdf::Vector3 noise_uniform;
  const sdf::Vector3 zeros3(0.0, 0.0, 0.0);

  // Retrieve the rest of the SDF parameters
  getSdfParam<std::string>(_sdf, "magnetometerTopic", magnetometer_topic_, mav_msgs::default_topics::MAGNETIC_FIELD);
  getSdfParam<double>(_sdf, "referenceMagNorth", ref_mag_north, kDefaultRefMagNorth);
  getSdfParam<double>(_sdf, "referenceMagEast", ref_mag_east, kDefaultRefMagEast);
  getSdfParam<double>(_sdf, "referenceMagDown", ref_mag_down, kDefaultRefMagDown);
  getSdfParam<sdf::Vector3>(_sdf, "noiseNormal", noise_normal, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "noiseUniform", noise_uniform, zeros3);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboMagnetometerPlugin::OnUpdate, this, _1));

  magnetometer_pub_ = node_handle_->advertise<sensor_msgs::MagneticField>(magnetometer_topic_, 1);

  mag_W_ = math::Vector3(ref_mag_north, ref_mag_east, ref_mag_down);

  // Fill magnetometer message
  magnetometer_message_.header.frame_id = frame_id_;
  magnetometer_message_.magnetic_field.x = mag_W_.x;
  magnetometer_message_.magnetic_field.y = mag_W_.y;
  magnetometer_message_.magnetic_field.z = mag_W_.z;
  magnetometer_message_.magnetic_field_covariance.fill(0.0);

  // Create the noise distributions
  noise_n_[0] = NormalDistribution(0, noise_normal.x);
  noise_n_[1] = NormalDistribution(0, noise_normal.y);
  noise_n_[2] = NormalDistribution(0, noise_normal.z);

  noise_u_[0] = UniformDistribution(-noise_uniform.x, noise_uniform.x);
  noise_u_[1] = UniformDistribution(-noise_uniform.y, noise_uniform.y);
  noise_u_[2] = UniformDistribution(-noise_uniform.z, noise_uniform.z);
}

void GazeboMagnetometerPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current pose and time from Gazebo
  math::Pose gazebo_pose = link_->GetWorldPose();
  common::Time current_time  = world_->GetSimTime();

  // Fill the magnetic field message header
  magnetometer_message_.header.stamp.sec = current_time.sec;
  magnetometer_message_.header.stamp.nsec = current_time.nsec;

  // Calculate the distortion in the orientation
  math::Quaternion q_noise;
  q_noise.SetFromEuler(noise_n_[0](random_generator_) + noise_u_[0](random_generator_),
          noise_n_[1](random_generator_) + noise_u_[1](random_generator_),
          noise_n_[2](random_generator_) + noise_u_[2](random_generator_));

  // Add distortion to current orientation
  math::Quaternion att = gazebo_pose.rot * q_noise;

  // Rotate the earth magnetic field into the inertial frame
  math::Vector3 field = att.RotateVectorReverse(mag_W_);

  // Fill the message values
  magnetometer_message_.magnetic_field.x = field.x;
  magnetometer_message_.magnetic_field.y = field.y;
  magnetometer_message_.magnetic_field.z = field.z;

  // Publish the message
  magnetometer_pub_.publish(magnetometer_message_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMagnetometerPlugin);
}
