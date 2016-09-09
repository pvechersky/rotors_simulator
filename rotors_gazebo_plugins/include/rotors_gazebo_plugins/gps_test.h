#ifndef ROTORS_GAZEBO_PLUGINS_GPS_TEST_H
#define ROTORS_GAZEBO_PLUGINS_GPS_TEST_H

#include <random>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {

class GpsTest : public ModelPlugin {
 public:
  GpsTest();
  virtual ~GpsTest();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void OnUpdate();

 private:
  // Pointer to the world
  physics::WorldPtr world_;

  // Pointer to the sensor link
  physics::LinkPtr link_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  double angle_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GPS_TEST_H
