#include "rotors_gazebo_plugins/gps_temp.h"

namespace gazebo {

GpsTemp::GpsTemp()
    : SensorPlugin(),
      node_handle_(0),
      random_generator_(random_device_()) {}

GpsTemp::~GpsTemp() {
  this->parent_sensor_->DisconnectUpdated(this->updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GpsTemp::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  // Store the pointer to the parent sensor.
#if GAZEBO_MAJOR_VERSION > 6
  parent_sensor_ = std::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);
  world_ = physics::get_world(parent_sensor_->WorldName());
#else
  parent_sensor_ = boost::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);
  world_ = physics::get_world(parent_sensor_->GetWorldName());
#endif

  // Retrieve the necessary parameters.
  std::string node_namespace;
  std::string link_name;

  if (_sdf->HasElement("robotNamespace"))
    node_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(node_namespace);

  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a linkName.\n";

  std::string frame_id = link_name;

  // Get the pointer to the link that holds the sensor.
  link_ = boost::dynamic_pointer_cast<physics::Link>(world_->GetByName(link_name));
  if (link_ == NULL)
    gzerr << "[gazebo_gps_plugin] Couldn't find specified link \"" << link_name << "\"\n";

  // Retrieve the rest of the SDF parameters.
  double hor_pos_std_dev;
  double ver_pos_std_dev;
  double hor_vel_std_dev;
  double ver_vel_std_dev;

  getSdfParam<std::string>(_sdf, "gpsTopic", gps_topic_,
                           mav_msgs::default_topics::GPS);
  getSdfParam<std::string>(_sdf, "groundSpeedTopic", ground_speed_topic_,
                           kDefaultGroundSpeedPubTopic);
  getSdfParam<double>(_sdf, "horPosStdDev", hor_pos_std_dev, kDefaultHorPosStdDev);
  getSdfParam<double>(_sdf, "verPosStdDev", ver_pos_std_dev, kDefaultVerPosStdDev);
  getSdfParam<double>(_sdf, "horVelStdDev", hor_vel_std_dev, kDefaultHorVelStdDev);
  getSdfParam<double>(_sdf, "verVelStdDev", ver_vel_std_dev, kDefaultVerVelStdDev);

  // Connect to the sensor update event.
  this->updateConnection_ =
      this->parent_sensor_->ConnectUpdated(
          boost::bind(&GpsTemp::OnUpdate, this));

  // Make sure the parent sensor is active.
  parent_sensor_->SetActive(true);

  common::SphericalCoordinatesPtr start_coordinates = world_->GetSphericalCoordinates();
  double ref_lat = start_coordinates->GetLatitudeReference().Degree();
  double ref_lon = start_coordinates->GetLongitudeReference().Degree();
  UTM::UTM(ref_lat, ref_lon, &start_easting_, &start_northing_);

  // Initialize the ROS publisher for sending gps location and ground speed.
  gps_pub_ = node_handle_->advertise<sensor_msgs::NavSatFix>(gps_topic_, 1);
  ground_speed_pub_ = node_handle_->advertise<geometry_msgs::TwistStamped>(ground_speed_topic_, 1);

  xyz_pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>("xyz_gazebo_gps", 1);

  // Initialize the normal distributions for ground speed.
  ground_speed_n_[0] = NormalDistribution(0, hor_vel_std_dev);
  ground_speed_n_[1] = NormalDistribution(0, hor_vel_std_dev);
  ground_speed_n_[2] = NormalDistribution(0, ver_vel_std_dev);

  // Fill the GPS message.
  gps_message_.header.frame_id = frame_id;
  gps_message_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  gps_message_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  gps_message_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
  gps_message_.position_covariance[0] = hor_pos_std_dev * hor_pos_std_dev;
  gps_message_.position_covariance[4] = hor_pos_std_dev * hor_pos_std_dev;
  gps_message_.position_covariance[8] = ver_pos_std_dev * ver_pos_std_dev;

  // Fill the ground speed message.
  ground_speed_message_.header.frame_id = frame_id;

  xyz_msg_.header.frame_id = frame_id;
}

void GpsTemp::OnUpdate() {
  // Get the time of the last measurement.
  common::Time current_time;

  // Get the linear velocity in the world frame.
  math::Vector3 W_ground_speed_W_L = link_->GetWorldLinearVel();

  // Apply noise to ground speed.
  W_ground_speed_W_L += math::Vector3(ground_speed_n_[0](random_generator_),
          ground_speed_n_[1](random_generator_),
          ground_speed_n_[2](random_generator_));

  // Fill the GPS message.
#if GAZEBO_MAJOR_VERSION > 6
  current_time = parent_sensor_->LastMeasurementTime();

  gps_message_.latitude = parent_sensor_->Latitude().Degree();
  gps_message_.longitude = parent_sensor_->Longitude().Degree();
  gps_message_.altitude = parent_sensor_->Altitude();
#else
  current_time = parent_sensor_->GetLastMeasurementTime();

  gps_message_.latitude = parent_sensor_->GetLatitude().Degree();
  gps_message_.longitude = parent_sensor_->GetLongitude().Degree();
  gps_message_.altitude = parent_sensor_->GetAltitude();
#endif
  gps_message_.header.stamp.sec = current_time.sec;
  gps_message_.header.stamp.nsec = current_time.nsec;

  // Fill the ground speed message.
  ground_speed_message_.twist.linear.x = W_ground_speed_W_L.x;
  ground_speed_message_.twist.linear.y = W_ground_speed_W_L.y;
  ground_speed_message_.twist.linear.z = W_ground_speed_W_L.z;
  ground_speed_message_.header.stamp.sec = current_time.sec;
  ground_speed_message_.header.stamp.nsec = current_time.nsec;

  // Publish the GPS message.
  gps_pub_.publish(gps_message_);

  // Publish the ground speed message.
  ground_speed_pub_.publish(ground_speed_message_);

  xyz_msg_.header.stamp.sec = current_time.sec;
  xyz_msg_.header.stamp.nsec = current_time.nsec;

  double x;
  double y;
  UTM::UTM(gps_message_.latitude, gps_message_.longitude, &x, &y);

  xyz_msg_.pose.position.x = (y - start_northing_);
  xyz_msg_.pose.position.y = -(x - start_easting_);

  xyz_pub_.publish(xyz_msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GpsTemp);
}
