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
#include <fstream>

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

  // Should a custom static wind field be used? Parameter to be set in techpod_base.xacro
  getSdfParam<bool>(_sdf, "customStaticWindField", custom_static_wind_field_, custom_static_wind_field_);

  if (!custom_static_wind_field_)
  {
    // Get the wind params from SDF.
    getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_, wind_force_mean_);
    getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_, wind_force_variance_);
    getSdfParam<math::Vector3>(_sdf, "windDirection", wind_direction_, wind_direction_);
    // Get the wind speed params from SDF
    getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_, wind_speed_mean_);
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
  }
  else
  {
    getSdfParam<math::Vector3>(_sdf, "windDirection", wind_direction_, wind_direction_);
    getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_, wind_speed_mean_);
    getSdfParam<double>(_sdf, "worldSideSize", world_side_size_, world_side_size_);
    getSdfParam<double>(_sdf, "worldHeight", world_height_, world_height_);
    getSdfParam<std::string>(_sdf, "windPath", wind_path_, wind_path_);
    // Read the txt file containing the wind field data, save it to a 2D array of math::Vector3 elements
    std::ifstream fin;
    fin.open(wind_path_);
    if (fin.is_open())
    {
      int i=0;
      math::Vector3 position;
      math::Vector3 wind;
      double count_x = 0;
      double value = 0;

      while (fin >> value)
      {
        position.x = value;
        fin >> position.y;
        fin >> position.z;
        fin >> wind.x;
        fin >> wind.y;
        fin >> wind.z;
        wind_field_[0].push_back(position);
        wind_field_[1].push_back(wind);
        // Calculate the number of points in the grid in z-direction
        if ( wind_field_[0][i].x == wind_field_[0][0].x && wind_field_[0][i].y == wind_field_[0][0].y )
          grid_size_z_++;
        if ( wind_field_[0][i].x == wind_field_[0][0].x )
          count_x++;
        i++;
      }
      ROS_INFO_STREAM("Wind field read successfully from text file.");

      // Get the number of points in the grid in x- and y-directions and find the resolutions
      double grid_size_y = count_x/grid_size_z_;
      double grid_size_x = wind_field_[0].size()/grid_size_y/grid_size_z_;
      res_x_ = world_side_size_/(grid_size_x-1);
      res_y_ = world_side_size_/(grid_size_y-1);

      // Print lines of wind field text file
      /*ROS_INFO_STREAM("Have written " << wind_field_[0].size() << " lines from wind field text file into an " << wind_field_[0].size() << "-dimensional vector.");
      for (int j=0; j<wind_field_[0].size();j++)
      {
        ROS_INFO_STREAM(wind_field_[0][j].x << " " << wind_field_[0][j].y << " " << wind_field_[0][j].z << " " << wind_field_[1][j].x << " " << wind_field_[1][j].y << " " << wind_field_[1][j].z);
      }*/

      fin.close();
    }
    else
    {
      gzerr << "[gazebo_wind_plugin] Could not open wind field text file.\n";
    }
  }


  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_ << "\".");

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

  math::Vector3 wind_velocity = math::Vector3(0,0,0);

  if (!custom_static_wind_field_) // Choose method
  {
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

    // Calculate the wind speed
    double wind_speed = wind_speed_mean_; // + wind_speed_n_(random_generator_);
    wind_velocity = wind_speed * wind_direction_;
  }
  else
  {
    // Get the current position of the aircraft in world coordinates
    math::Vector3 link_position = link_->GetWorldPose().pos;

    //Determine the wind velocity at the aircraft position if within world bounds
    if ( !(fabs(link_position.x)>(world_side_size_/2.0) || fabs(link_position.y)>(world_side_size_/2.0) || link_position.z > (world_height_*2.0) || link_position.z < 0) )
    {
      // Create a vector containing the coordinates of 8 points corresponding to the vertices of the volume enclosing the link and the 5 points for interpolation
      // Containing as well the wind velocity value for each point
      std::vector<math::Vector3> wind_field_cells[2];
      for (int i = 0; i < 14; i++)
      {
        for (int j=0; j<2; j++)
          wind_field_cells[j].push_back(math::Vector3(0, 0, 0));
      }

      // Set the x- and y-coordinates of the points
      wind_field_cells[0][0].x= wind_field_cells[0][3].x = wind_field_cells[0][4].x = wind_field_cells[0][7].x = wind_field_cells[0][8].x = wind_field_cells[0][11].x = wind_field_cells[0][12].x = floor(link_position.x/res_x_)*res_x_;
      wind_field_cells[0][1].x = wind_field_cells[0][2].x = wind_field_cells[0][5].x = wind_field_cells[0][6].x = wind_field_cells[0][9].x = wind_field_cells[0][10].x = wind_field_cells[0][13].x = ceil(link_position.x/res_x_)*res_x_;
      wind_field_cells[0][0].y = wind_field_cells[0][1].y = wind_field_cells[0][2].y = wind_field_cells[0][3].y = wind_field_cells[0][8].y = wind_field_cells[0][9].y = floor(link_position.y/res_y_)*res_y_;
      wind_field_cells[0][4].y = wind_field_cells[0][5].y = wind_field_cells[0][6].y = wind_field_cells[0][7].y = wind_field_cells[0][10].y = wind_field_cells[0][11].y = ceil(link_position.y/res_y_)*res_y_;
      wind_field_cells[0][12].y = wind_field_cells[0][13].y = link_position.y;

      // Determine the z-coordinates of the points
      // Initialize needed variables
      double z_0_min = world_height_*2.0;
      double z_0_max = 0;
      double z_1_min = world_height_*2.0;
      double z_1_max = 0;
      double z_4_min = world_height_*2.0;
      double z_4_max = 0;
      double z_5_min = world_height_*2.0;
      double z_5_max = 0;
      double res_z_0 = 0.0;
      double res_z_1 = 0.0;
      double res_z_4 = 0.0;
      double res_z_5 = 0.0;

      // Read through txt file for given x-y pairs, find their resolution in z-direction comparing biggest and smallest altitude

      // TO DO : Change calculation of z to be only dependent of file values, not calculated values!
      // E.g., save all z-values corresponding to a x-y-pair into a vector, sort them from biggest to smallest,
      // browse through to find the one where z[i] < link_position.z && z[i+1] > link_position.z

      for (int i = 0; i < wind_field_[0].size(); i++)
      {
        //Determine z resolution above point 0
        if (wind_field_[0][i].x == wind_field_cells[0][0].x && wind_field_[0][i].y == wind_field_cells[0][0].y)
        {
          if (wind_field_[0][i].z < z_0_min)
            z_0_min = wind_field_[0][i].z;
          if (wind_field_[0][i].z > z_0_min)
            z_0_max = wind_field_[0][i].z;
        }
        res_z_0 = (z_0_max - z_0_min)/(grid_size_z_-1);
        //Determine z resolution above point 1
        if (wind_field_[0][i].x == wind_field_cells[0][1].x && wind_field_[0][i].y == wind_field_cells[0][1].y)
        {
          if (wind_field_[0][i].z < z_1_min)
            z_1_min = wind_field_[0][i].z;
          if (wind_field_[0][i].z > z_1_min)
            z_1_max = wind_field_[0][i].z;
        }
        res_z_1 = (z_1_max - z_1_min)/(grid_size_z_-1);
        //Determine z resolution above point 4
        if (wind_field_[0][i].x == wind_field_cells[0][4].x && wind_field_[0][i].y == wind_field_cells[0][4].y)
        {
          if (wind_field_[0][i].z < z_4_min)
            z_4_min = wind_field_[0][i].z;
          if (wind_field_[0][i].z > z_4_min)
            z_4_max = wind_field_[0][i].z;
        }
        res_z_4 = (z_4_max - z_4_min)/(grid_size_z_-1);
        //Determine z resolution above point 5
        if (wind_field_[0][i].x == wind_field_cells[0][5].x && wind_field_[0][i].y == wind_field_cells[0][5].y)
        {
          if (wind_field_[0][i].z < z_5_min)
            z_5_min = wind_field_[0][i].z;
          if (wind_field_[0][i].z > z_5_min)
            z_5_max = wind_field_[0][i].z;
        }
        res_z_5 = (z_5_max - z_5_min)/(grid_size_z_-1);
      }

      // Set z-coordinates. Precision of txt file is two decimals after the comma
      wind_field_cells[0][0].z =  floor(link_position.z/res_z_0)*res_z_0;
      wind_field_cells[0][3].z =  ceil(link_position.z/res_z_0)*res_z_0;
      wind_field_cells[0][1].z =  floor(link_position.z/res_z_1)*res_z_1;
      wind_field_cells[0][2].z =  ceil(link_position.z/res_z_1)*res_z_1;
      wind_field_cells[0][4].z =  floor(link_position.z/res_z_4)*res_z_4;
      wind_field_cells[0][7].z =  ceil(link_position.z/res_z_4)*res_z_4;
      wind_field_cells[0][5].z =  floor(link_position.z/res_z_5)*res_z_5;
      wind_field_cells[0][6].z =  ceil(link_position.z/res_z_5)*res_z_5;
      /*wind_field_cells[0][0].z =  round(floor(link_position.z/res_z_0)*res_z_0*100.0)/100.0;
      wind_field_cells[0][3].z =  round(ceil(link_position.z/res_z_0)*res_z_0*100.0)/100.0;
      wind_field_cells[0][1].z =  round(floor(link_position.z/res_z_1)*res_z_1*100.0)/100.0;
      wind_field_cells[0][2].z =  round(ceil(link_position.z/res_z_1)*res_z_1*100.0)/100.0;
      wind_field_cells[0][4].z =  round(floor(link_position.z/res_z_4)*res_z_4*100.0)/100.0;
      wind_field_cells[0][7].z =  round(ceil(link_position.z/res_z_4)*res_z_4*100.0)/100.0;
      wind_field_cells[0][5].z =  round(floor(link_position.z/res_z_5)*res_z_5*100.0)/100.0;
      wind_field_cells[0][6].z =  round(ceil(link_position.z/res_z_5)*res_z_5*100.0)/100.0;*/
      wind_field_cells[0][8].z =  wind_field_cells[0][9].z =  wind_field_cells[0][10].z =  wind_field_cells[0][11].z =  wind_field_cells[0][12].z =  wind_field_cells[0][13].z = link_position.z;

      // Extract velocity components of each of the 8 given vertices. Problem: rounded values in file do not correspond to calculated values. Sensitive to geometry...
      for (int i = 0; i < wind_field_[1].size(); i++)
      {
        for (int j = 0; j < 8; j++)
        {
          if (wind_field_[0][i] == wind_field_cells[0][j])
              wind_field_cells[1][j] = wind_field_[1][i];
        }
      }

      // Interpolate three times linearly to find the value of the wind velocity at the given position
      wind_field_cells[1][8] = wind_field_cells[1][0] + (wind_field_cells[1][3] - wind_field_cells[1][0])/(wind_field_cells[0][3].z - wind_field_cells[0][0].z)*(wind_field_cells[0][8].z - wind_field_cells[0][0].z);
      wind_field_cells[1][9] = wind_field_cells[1][1] + (wind_field_cells[1][2] - wind_field_cells[1][1])/(wind_field_cells[0][2].z - wind_field_cells[0][1].z)*(wind_field_cells[0][9].z - wind_field_cells[0][1].z);
      wind_field_cells[1][10] = wind_field_cells[1][5] + (wind_field_cells[1][6] - wind_field_cells[1][5])/(wind_field_cells[0][6].z - wind_field_cells[0][5].z)*(wind_field_cells[0][10].z - wind_field_cells[0][5].z);
      wind_field_cells[1][11] = wind_field_cells[1][4] + (wind_field_cells[1][7] - wind_field_cells[1][4])/(wind_field_cells[0][7].z - wind_field_cells[0][4].z)*(wind_field_cells[0][11].z - wind_field_cells[0][4].z);

      wind_field_cells[1][12] = wind_field_cells[1][8] + (wind_field_cells[1][11] - wind_field_cells[1][8])/(wind_field_cells[0][11].y - wind_field_cells[0][8].y)*(wind_field_cells[0][12].y - wind_field_cells[0][8].y);
      wind_field_cells[1][13] = wind_field_cells[1][9] + (wind_field_cells[1][10] - wind_field_cells[1][9])/(wind_field_cells[0][10].y - wind_field_cells[0][9].y)*(wind_field_cells[0][13].y - wind_field_cells[0][9].y);

      wind_velocity = wind_field_cells[1][12] + (wind_field_cells[1][13] - wind_field_cells[1][12])/(wind_field_cells[0][13].x - wind_field_cells[0][12].x)*(link_position.x - wind_field_cells[0][12].x);
    }
    else
    {
      wind_velocity = wind_speed_mean_ * wind_direction_;
    }
  }

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
