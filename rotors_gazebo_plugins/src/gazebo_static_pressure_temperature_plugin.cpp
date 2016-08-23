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

#include "rotors_gazebo_plugins/gazebo_static_pressure_temperature_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboStaticPressureTemperaturePlugin::GazeboStaticPressureTemperaturePlugin()
    : ModelPlugin(),
      node_handle_(0),
      random_generator_(random_device_()) {
}

GazeboStaticPressureTemperaturePlugin::~GazeboStaticPressureTemperaturePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboStaticPressureTemperaturePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model and the world.
  model_ = _model;
  world_ = model_->GetWorld();

  // Use the robot namespace to create the node handle.
  std::string node_namespace;
  if (_sdf->HasElement("robotNamespace"))
    node_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_static_pressure_temperature_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(node_namespace);

  // Use the link name as the frame id.
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_static_pressure_temperature_plugin] Please specify a linkName.\n";
  // Get the pointer to the link.
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_static_pressure_temperature_plugin] Couldn't find specified link \"" << link_name << "\".");

  std::string frame_id = link_name;

  // Retrieve the rest of the SDF parameters.
  std::string pressure_topic;
  std::string temperature_topic;

  double pressure_var;
  double temperature_var;

  getSdfParam<std::string>(_sdf, "pressureTopic", pressure_topic,
                           kDefaultPressurePubTopic);
  getSdfParam<std::string>(_sdf, "temperatureTopic", temperature_topic,
                           kDefaultTemperaturePubTopic);
  getSdfParam<double>(_sdf, "referenceAltitude", ref_alt_,
                      world_->GetSphericalCoordinates()->GetElevationReference());
  getSdfParam<double>(_sdf, "pressureVariance", pressure_var,
                      kDefaultPressureVar);
  getSdfParam<double>(_sdf, "temperatureVariance", temperature_var,
                      kDefaultTemperatureVar);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboStaticPressureTemperaturePlugin::OnUpdate, this, _1));

  pressure_pub_ = node_handle_->advertise<sensor_msgs::FluidPressure>(pressure_topic, 1);
  temperature_pub_ = node_handle_->advertise<sensor_msgs::Temperature>(temperature_topic, 1);

  // Create the normal noise distribution objects.
  pressure_n_ = NormalDistribution(0, pressure_var);
  temperature_n_ = NormalDistribution(0, temperature_var);

  // Fill the pressure message.
  pressure_message_.header.frame_id = frame_id;
  pressure_message_.variance = pressure_var;

  // Fill the temperature message.
  temperature_message_.header.frame_id = frame_id;
  temperature_message_.variance = temperature_var;
}

void GazeboStaticPressureTemperaturePlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();

  // Compute the geopotential altitude.
  double z = ref_alt_ + link_->GetWorldPose().pos.z;
  double h = kR0 * z / (kR0 + z);

  // Compute the temperature at the current altitude.
  double t = kT0 - kTl * h;

  // Compute the current static air pressure.
  double p = kP0 * exp(kAs * log(kT0 / t));

  // Fill the pressure message.
  pressure_message_.fluid_pressure = p + pressure_n_(random_generator_);
  pressure_message_.header.stamp.sec = current_time.sec;
  pressure_message_.header.stamp.nsec = current_time.nsec;

  // Fill the temperature message.
  temperature_message_.temperature = t + temperature_n_(random_generator_);
  temperature_message_.header.stamp.sec = current_time.sec;
  temperature_message_.header.stamp.nsec = current_time.nsec;

  // Publish the pressure message.
  pressure_pub_.publish(pressure_message_);

  // Publish the temperature message.
  temperature_pub_.publish(temperature_message_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboStaticPressureTemperaturePlugin);
}
