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

#include "rotors_gazebo_plugins/gazebo_control_surface_plugin.h"

namespace gazebo {

GazeboControlSurfacePlugin::GazeboControlSurfacePlugin()
    : ModelPlugin(),
      node_handle_(0) {}

GazeboControlSurfacePlugin::~GazeboControlSurfacePlugin() {
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboControlSurfacePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Get the robot namespace.
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    node_handle_ = new ros::NodeHandle(namespace_);
  }
  else
    gzerr << "[gazebo_control_surface_plugin] Please specify a robotNamespace.\n";

  // Get the pointer to the joint.
  if (_sdf->HasElement("jointName")) {
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  }
  else
    gzerr << "[gazebo_control_surface_plugin] Please specify a jointName.\n";

  // Get the type of the control surface.
  if (_sdf->HasElement("surfaceType")) {
    surface_type_ = _sdf->GetElement("surfaceType")->Get<std::string>();
    if (!(surface_type_ == "aileron" || surface_type_ == "elevator" ||
          surface_type_ == "rudder" || surface_type_ == "flap"))
      gzerr << "[gazebo_control_surface_plugin] " <<
               "Please only use 'aileron', 'elevator', 'rudder', or 'flap' as surfaceType.\n";
  }
  else
    gzerr << "[gazebo_control_surface_plugin] " <<
             "Please specify the surface type ('aileron', 'elevator', 'rudder', or 'flap').\n";

  // Get the side of the airplane which the control surface is on.
  if (surface_type_ == "aileron" || surface_type_ == "flap") {
    if (_sdf->HasElement("surfaceSide")) {
        surface_side_ = _sdf->GetElement("surfaceSide")->Get<std::string>();
        if (!(surface_side_ == "left" || surface_side_ == "right"))
          gzerr << "[gazebo_control_surface_plugin] " <<
                   "Please only use 'left' or 'right' as surfaceSide.\n";
    }
    else
      gzerr << "[gazebo_control_surface_plugin] " <<
               "Please specify the surface side ('left' or 'right') for ailerons and flaps.\n";
  }
  else
    surface_side_ = "";

  // Get the minimum and maximum deflection angles.
  getSdfParam<double>(_sdf, "angleMin", angle_min_, kDefaultAngleMin);
  getSdfParam<double>(_sdf, "angleMax", angle_max_, kDefaultAngleMax);

  // Create the client for registering the control surface info.
  register_control_surface_client_ =
          node_handle_->serviceClient<rotors_comm::RegisterControlSurface>(
              "register_control_surface");

  // Generate the request for registering the control surface info.
  rotors_comm::RegisterControlSurface control_surface_info;
  control_surface_info.request.joint_name = joint_name_;
  control_surface_info.request.surface_type = surface_type_;
  control_surface_info.request.surface_side = surface_side_;
  control_surface_info.request.angle_min = angle_min_;
  control_surface_info.request.angle_max = angle_max_;

  // Send the control surface info.
  if (!register_control_surface_client_.call(control_surface_info))
    gzerr << "[gazebo_control_surface_plugin] Unable to register the control surface info.\n";
}

GZ_REGISTER_MODEL_PLUGIN(GazeboControlSurfacePlugin);
}
