#ifndef AERO_VISUAL_PLUGIN_H
#define AERO_VISUAL_PLUGIN_H

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

class GAZEBO_VISIBLE AeroVisualPlugin : public VisualPlugin {
 public:
  AeroVisualPlugin();
  virtual ~AeroVisualPlugin();

  void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

  void OnUpdate();

  void ForceCallback(const geometry_msgs::WrenchStampedConstPtr& force_msg);

 private:
  ros::NodeHandle* rosnode_;
  ros::Subscriber force_sub_;

  rendering::VisualPtr visual_;
  rendering::VisualPtr force_visual_;

  rendering::DynamicLines *line;

  event::ConnectionPtr update_connection_;
};
}

#endif // AERO_VISUAL_PLUGIN_H
