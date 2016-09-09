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

#include "rotors_gazebo_plugins/gazebo_imu_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboImuPlugin::GazeboImuPlugin()
    : ModelPlugin(),
      node_handle_(0),
      random_generator_(std::chrono::system_clock::now().time_since_epoch().count()),
      velocity_prev_W_(0, 0, 0),
      roll_(0.0),
      pitch_(0.0),
      yaw_(0.0) {}

GazeboImuPlugin::~GazeboImuPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // Set the simulation time to current wall time
  ros::Time current_ros_time = ros::Time::now();
  common::Time new_sim_time(current_ros_time.sec, current_ros_time.nsec);
  world_->SetSimTime(new_sim_time);

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  frame_id_ = link_name_;

  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_,
                           mav_msgs::default_topics::IMU);
  getSdfParam<double>(_sdf, "gyroscopeNoiseDensity",
                      imu_parameters_.gyroscope_noise_density,
                      imu_parameters_.gyroscope_noise_density);
  getSdfParam<double>(_sdf, "gyroscopeBiasRandomWalk",
                      imu_parameters_.gyroscope_random_walk,
                      imu_parameters_.gyroscope_random_walk);
  getSdfParam<double>(_sdf, "gyroscopeBiasCorrelationTime",
                      imu_parameters_.gyroscope_bias_correlation_time,
                      imu_parameters_.gyroscope_bias_correlation_time);
  assert(imu_parameters_.gyroscope_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "gyroscopeTurnOnBiasSigma",
                      imu_parameters_.gyroscope_turn_on_bias_sigma,
                      imu_parameters_.gyroscope_turn_on_bias_sigma);
  getSdfParam<double>(_sdf, "accelerometerNoiseDensity",
                      imu_parameters_.accelerometer_noise_density,
                      imu_parameters_.accelerometer_noise_density);
  getSdfParam<double>(_sdf, "accelerometerRandomWalk",
                      imu_parameters_.accelerometer_random_walk,
                      imu_parameters_.accelerometer_random_walk);
  getSdfParam<double>(_sdf, "accelerometerBiasCorrelationTime",
                      imu_parameters_.accelerometer_bias_correlation_time,
                      imu_parameters_.accelerometer_bias_correlation_time);
  assert(imu_parameters_.accelerometer_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "accelerometerTurnOnBiasSigma",
                      imu_parameters_.accelerometer_turn_on_bias_sigma,
                      imu_parameters_.accelerometer_turn_on_bias_sigma);

  last_time_ = world_->GetSimTime();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboImuPlugin::OnUpdate, this, _1));

  imu_pub_ = node_handle_->advertise<sensor_msgs::Imu>(imu_topic_, 1);

  // Fill imu message.
  imu_message_.header.frame_id = frame_id_;
  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.
  // Angular velocity measurement covariance.
  imu_message_.angular_velocity_covariance[0] =
      imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_message_.angular_velocity_covariance[4] =
      imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_message_.angular_velocity_covariance[8] =
      imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  // Linear acceleration measurement covariance.
  imu_message_.linear_acceleration_covariance[0] =
      imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_message_.linear_acceleration_covariance[4] =
      imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_message_.linear_acceleration_covariance[8] =
      imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  // Orientation estimate covariance (no estimate provided).
  imu_message_.orientation_covariance[0] = -1.0;

  gravity_W_ = world_->GetPhysicsEngine()->GetGravity();
  imu_parameters_.gravity_magnitude = gravity_W_.GetLength();

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  double sigma_bon_g = imu_parameters_.gyroscope_turn_on_bias_sigma;
  double sigma_bon_a = imu_parameters_.accelerometer_turn_on_bias_sigma;
  for (int i = 0; i < 3; ++i) {
      gyroscope_turn_on_bias_[i] =
          sigma_bon_g * standard_normal_distribution_(random_generator_);
      accelerometer_turn_on_bias_[i] =
          sigma_bon_a * standard_normal_distribution_(random_generator_);
  }

  // TODO(nikolicj) incorporate steady-state covariance of bias process
  gyroscope_bias_.setZero();
  accelerometer_bias_.setZero();
}

/// \brief This function adds noise to acceleration and angular rates for
///        accelerometer and gyroscope measurement simulation.
void GazeboImuPlugin::addNoise(Eigen::Vector3d* linear_acceleration,
                               Eigen::Vector3d* angular_velocity,
                               const double dt) {
  ROS_ASSERT(linear_acceleration != nullptr);
  ROS_ASSERT(angular_velocity != nullptr);
  ROS_ASSERT(dt > 0.0);

  // Gyroscope
  double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
  double sigma_b_g = imu_parameters_.gyroscope_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d =
      sqrt( - sigma_b_g * sigma_b_g * tau_g / 2.0 *
      (exp(-2.0 * dt / tau_g) - 1.0));
  // Compute state-transition.
  double phi_g_d = exp(-1.0 / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i) {
    gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +
        sigma_b_g_d * standard_normal_distribution_(random_generator_);
    (*angular_velocity)[i] = (*angular_velocity)[i] +
        gyroscope_bias_[i] +
        sigma_g_d * standard_normal_distribution_(random_generator_) +
        gyroscope_turn_on_bias_[i];
  }

  // Accelerometer
  double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
  double sigma_b_a = imu_parameters_.accelerometer_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d =
      sqrt( - sigma_b_a * sigma_b_a * tau_a / 2.0 *
      (exp(-2.0 * dt / tau_a) - 1.0));
  // Compute state-transition.
  double phi_a_d = exp(-1.0 / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  for (int i = 0; i < 3; ++i) {
    accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +
        sigma_b_a_d * standard_normal_distribution_(random_generator_);
    (*linear_acceleration)[i] = (*linear_acceleration)[i] +
        accelerometer_bias_[i] +
        sigma_a_d * standard_normal_distribution_(random_generator_) +
        accelerometer_turn_on_bias_[i];
  }
}

void GazeboImuPlugin::ComplementaryFilter(const Eigen::Vector3d linear_acceleration,
                                          const Eigen::Vector3d angular_velocity,
                                          double dt) {
  roll_ += angular_velocity[0] * dt;
  pitch_ += angular_velocity[1] * dt;

  double roll_acc = atan2(linear_acceleration[1], linear_acceleration[2]);
  double pitch_acc = atan2(linear_acceleration[0], linear_acceleration[2]);

  roll_ = 0.98 * roll_ + 0.02 * roll_acc;
  pitch_ = 0.98 * pitch_ + 0.02 * pitch_acc;
}

Eigen::Quaterniond GazeboImuPlugin::QuaternionFromEuler(Eigen::Vector3d euler) {
  Eigen::AngleAxisd roll_angle(euler[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(euler[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(euler[2], Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond q = roll_angle * pitch_angle * yaw_angle;
  return q;
}

// This gets called by the world update start event.
void GazeboImuPlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;

  math::Pose T_W_I = link_->GetWorldPose(); //TODO(burrimi): Check tf.
  math::Quaternion C_W_I = T_W_I.rot;

  math::Vector3 acceleration_I;

#if GAZEBO_MAJOR_VERSION < 5
  math::Vector3 velocity_current_W = link_->GetWorldLinearVel();
  // link_->GetRelativeLinearAccel() does not work sometimes with old gazebo versions.
  // This issue is solved in gazebo 5.
  math::Vector3 acceleration = (velocity_current_W - velocity_prev_W_) / dt;
  acceleration_I = C_W_I.RotateVectorReverse(acceleration - gravity_W_);

  velocity_prev_W_ = velocity_current_W;
#else
  acceleration_I = link_->GetRelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_);
#endif

  math::Vector3 angular_vel_I = link_->GetRelativeAngularVel();

  Eigen::Vector3d linear_acceleration_I(acceleration_I.x,
                                        acceleration_I.y,
                                        acceleration_I.z);
  Eigen::Vector3d angular_velocity_I(angular_vel_I.x,
                                     angular_vel_I.y,
                                     angular_vel_I.z);

  addNoise(&linear_acceleration_I, &angular_velocity_I, dt);

  //ComplementaryFilter(linear_acceleration_I, angular_velocity_I, dt);

  /*math::Vector3 acceleration_drift =
      C_W_I.RotateVector(math::Vector3(accelerometer_bias_[0],
                                       accelerometer_bias_[1],
                                       accelerometer_bias_[2]));
  double yaw_error = 0.0; //yawModel.getCurrentBias();

  // Yaw error quaternion
  math::Quaternion yaw_error_quat(cos(yaw_error / 2),
                                  0.0,
                                  0.0,
                                  sin(yaw_error / 2));

  // Roll and pitch error quaternion
  math::Quaternion roll_pitch_error_quat(1.0,
                                         0.5 * acceleration_drift.y / imu_parameters_.gravity_magnitude,
                                         0.5 * -acceleration_drift.x / imu_parameters_.gravity_magnitude,
                                         0.0);

  math::Quaternion orientation_error(yaw_error_quat * roll_pitch_error_quat);
  orientation_error.Normalize();
  C_W_I = orientation_error * C_W_I;*/

  //Eigen::Vector3d theta;
  //theta << roll_, pitch_, yaw_;
  //Eigen::Quaterniond orientation = QuaternionFromEuler(theta);

  // Fill IMU message.
  imu_message_.header.stamp.sec = current_time.sec;
  imu_message_.header.stamp.nsec = current_time.nsec;

  imu_message_.orientation.w = C_W_I.w;
  imu_message_.orientation.x = C_W_I.x;
  imu_message_.orientation.y = C_W_I.y;
  imu_message_.orientation.z = C_W_I.z;

  /*imu_message_.orientation.w = orientation.w();
  imu_message_.orientation.x = orientation.x();
  imu_message_.orientation.y = orientation.y();
  imu_message_.orientation.z = orientation.z();*/

  imu_message_.linear_acceleration.x = linear_acceleration_I[0];
  imu_message_.linear_acceleration.y = linear_acceleration_I[1];
  imu_message_.linear_acceleration.z = linear_acceleration_I[2];
  imu_message_.angular_velocity.x = angular_velocity_I[0];
  imu_message_.angular_velocity.y = angular_velocity_I[1];
  imu_message_.angular_velocity.z = angular_velocity_I[2];

  imu_pub_.publish(imu_message_);

}

GZ_REGISTER_MODEL_PLUGIN(GazeboImuPlugin);
}
