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

#include "rotors_hil_interface/hil_interface_node.h"

namespace rotors_hil {

HilInterfaceNode::HilInterfaceNode() :
    rate_(kDefaultHilFrequency) {
  ros::NodeHandle pnh("~");

  bool sensor_level_hil;
  double hil_frequency;
  std::string mavlink_pub_topic;

  pnh.param("sensor_level_hil", sensor_level_hil, kDefaultSensorLevelHil);
  pnh.param("hil_frequency", hil_frequency, kDefaultHilFrequency);
  pnh.param("mavlink_pub_topic", mavlink_pub_topic, kDefaultMavlinkPubTopic);

  if (sensor_level_hil)
    hil_interface_ = std::unique_ptr<HilSensorLevelInterface>();
  else
    hil_interface_ = std::unique_ptr<HilStateLevelInterface>();

  rate_ = ros::Rate(hil_frequency);

  mavlink_pub_ = nh_.advertise<mavros_msgs::Mavlink>(mavlink_pub_topic, 5);
}

HilInterfaceNode::~HilInterfaceNode() {
}

void HilInterfaceNode::MainTask() {
  while (ros::ok()) {
    std::vector<mavros_msgs::Mavlink> hil_msgs = hil_interface_->CollectData();

    while (!hil_msgs.empty()) {
      mavlink_pub_.publish(hil_msgs.back());
      hil_msgs.pop_back();
    }

    ros::spinOnce();
    rate_.sleep();
  }
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_hil_interface_node");
  rotors_hil::HilInterfaceNode hil_interface_node;

  hil_interface_node.MainTask();

  return 0;
}
