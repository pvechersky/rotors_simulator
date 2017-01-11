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


#include "rotors_gazebo_plugins/gazebo_wind_plugin.h"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>

#include <math.h>

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}
;

void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<math::Vector3>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  //Elements of function: getSdfParam<type>(sdf element, name of parameter in sdf file, name of parameter in this class, default value)
  getSdfParam<std::string>(_sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedPubTopic", wind_speed_pub_topic_, wind_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  // Get the wind params from SDF.
  getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_, wind_force_mean_);
  getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_, wind_force_variance_);
  getSdfParam<math::Vector3>(_sdf, "windDirection", wind_direction_, wind_direction_);
  // Get the wind speed params from SDF
  getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_, wind_speed_mean_); //  bool dummy = getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_, wind_speed_mean_);
  /*  For debugging:
  ROS_INFO_STREAM("wind_speed_mean_ is " << wind_speed_mean_);
  ROS_INFO_STREAM("value of getSdfParam is " << dummy);
  */
  getSdfParam<double>(_sdf, "windSpeedVariance", wind_speed_variance_, wind_speed_variance_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(_sdf, "windGustForceMean", wind_gust_force_mean_, wind_gust_force_mean_);
  getSdfParam<double>(_sdf, "windGustForceVariance", wind_gust_force_variance_, wind_gust_force_variance_);
  getSdfParam<math::Vector3>(_sdf, "windGustDirection", wind_gust_direction_, wind_gust_direction_);

  wind_direction_.Normalize();
  wind_gust_direction_.Normalize();
  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);

  wind_force_n_ = NormalDistribution(0, sqrt(wind_force_variance_));
  wind_gust_force_n_ = NormalDistribution(0, sqrt(wind_gust_force_variance_));
  wind_speed_n_ = NormalDistribution(0, sqrt(wind_speed_variance_));

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  ////////-------------------------TO DO NICOLAS--------------------------------

  // Read the txt file containing the wind field data, save it to an array




  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->advertise<geometry_msgs::WrenchStamped>(wind_pub_topic_, 1);
  wind_speed_pub_ = node_handle_->advertise<rotors_comm::WindSpeed>(wind_speed_pub_topic_, 1);
}






// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
  common::Time now = world_->GetSimTime();

  // Calculate the wind force.
  double wind_strength = wind_force_mean_ + wind_force_n_(random_generator_);
  math::Vector3 wind = wind_strength * wind_direction_;
  // Apply a force from the constant wind to the link.
  link_->AddForceAtRelativePosition(wind, xyz_offset_);

  math::Vector3 wind_gust(0, 0, 0);
  // Calculate the wind gust force.
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    double wind_gust_strength = wind_gust_force_mean_ + wind_gust_force_n_(random_generator_);
    wind_gust = wind_gust_strength * wind_gust_direction_;
    // Apply a force from the wind gust to the link.
    link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);
  }

  geometry_msgs::WrenchStamped wrench_msg;

  wrench_msg.header.frame_id = frame_id_;
  wrench_msg.header.stamp.sec = now.sec;
  wrench_msg.header.stamp.nsec = now.nsec;
  wrench_msg.wrench.force.x = wind.x + wind_gust.x;
  wrench_msg.wrench.force.y = wind.y + wind_gust.y;
  wrench_msg.wrench.force.z = wind.z + wind_gust.z;
  wrench_msg.wrench.torque.x = 0;
  wrench_msg.wrench.torque.y = 0;
  wrench_msg.wrench.torque.z = 0;

  wind_pub_.publish(wrench_msg);

  ////////-------------------------TO DO NICOLAS--------------------------------

  /* Implement the static wind field here instead of what is below:
     Step 0: read the text file and save it into a 3D array (done when loading plugin)
     Step 1: get the current position of the aircraft
     Step 2: identify the vertices of the cell enclosing the aircraft
     Step 3: read the wind velocity for these vertices from the array
     Step 4: Interpolate the wind velocity
     Step 5: set wind_speed_msg.velocity.x/y/z equal to the interpolated values in the table

     Better: subscribe to a topic that publishes the lookup table, save it.
     This way, wind field can be updated during the simulation (dynamic field).
     Subscriber implemented outside of this function?
     This exceeds the scope of the thesis.
  */

  // Set geometrical values (to be tuned according to grayscale image used to generate world)
  double grayscale_size = 257;
  double world_size = 4000;
  double hemicylinder_radius = 13;
  double world_resolution = world_size/grayscale_size;
  double world_height = world_resolution*hemicylinder_radius;
  double hemicylinder_x_position = world_resolution*128;

  // Get the current position of the aircraft in world coordinates
  math::Vector3 link_position = link_->GetWorldPose().pos;

  //Determine the wind velocity at the aircraft position
  math::Vector3 wind_velocity = math::Vector3(0, 0, 0);

  if ( fabs(link_position.x)>(world_size/2.0) || fabs(link_position.y)>(world_size/2.0) || link_position.z > (world_height*2.0) || link_position.z < 0 )
  {
    wind_velocity = math::Vector3(0,0,0);
  }
  else
  {
    // Read the 8 points corresponding to the vertices of the volume enclosing the link
    // and the 5 for interpolation
    math::Vector3 point_0 = math::Vector3(0,0,0);
    math::Vector3 point_1 = math::Vector3(0,0,0);
    math::Vector3 point_2 = math::Vector3(0,0,0);
    math::Vector3 point_3 = math::Vector3(0,0,0);
    math::Vector3 point_4 = math::Vector3(0,0,0);
    math::Vector3 point_5 = math::Vector3(0,0,0);
    math::Vector3 point_6 = math::Vector3(0,0,0);
    math::Vector3 point_7 = math::Vector3(0,0,0);

    math::Vector3 point_8 = math::Vector3(0,0,0);
    math::Vector3 point_9 = math::Vector3(0,0,0);
    math::Vector3 point_10 = math::Vector3(0,0,0);
    math::Vector3 point_11 = math::Vector3(0,0,0);
    math::Vector3 point_12 = math::Vector3(0,0,0);
    math::Vector3 point_13 = math::Vector3(0,0,0);

    point_0.x = point_3.x = point_4.x = point_7.x = point_8.x = point_11.x = point_12.x = floor(link_position.x/world_resolution)*world_resolution;
    point_1.x = point_2.x = point_5.x = point_6.x = point_9.x = point_10.x = point_13.x = ceil(link_position.x/world_resolution)*world_resolution;
    point_0.x = point_1.x = point_2.x = point_3.x = point_8.x = point_9.x = floor(link_position.y/world_resolution)*world_resolution;
    point_4.x = point_5.x = point_6.x = point_7.x = point_10.x = point_11.x = ceil(link_position.y/world_resolution)*world_resolution;
    point_12.y = point_13.y = link_position.y;
    // Determine their z-coordinates! Read through txt file for given x-y pairs,
    // identify lines with z just below/just above link_position.z

    // Read through txt file to extract velocity components of each of the 8 given vertices
    math::Vector3 wind_velocity_0 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_1 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_2 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_3 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_4 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_5 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_6 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_7 = math::Vector3(0,0,0);

    math::Vector3 wind_velocity_8 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_9 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_10 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_11 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_12 = math::Vector3(0,0,0);
    math::Vector3 wind_velocity_13 = math::Vector3(0,0,0);

    // Interpolate linearly to find the value of the wind velocity at the given position


    wind_velocity = wind_speed_mean_*wind_direction_; // = ... (interpolation)



  }

  /*// Calculate the wind speed
  double wind_speed = wind_speed_mean_; // + wind_speed_n_(random_generator_);
  math::Vector3 wind_velocity = wind_speed * wind_direction_;*/

  // Publish the wind speed
  rotors_comm::WindSpeed wind_speed_msg;

  wind_speed_msg.header.frame_id = frame_id_;
  wind_speed_msg.header.stamp.sec = now.sec;
  wind_speed_msg.header.stamp.nsec = now.nsec;
  wind_speed_msg.velocity.x = wind_velocity.x;
  wind_speed_msg.velocity.y = wind_velocity.y;
  wind_speed_msg.velocity.z = wind_velocity.z;

  wind_speed_pub_.publish(wind_speed_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);
}
