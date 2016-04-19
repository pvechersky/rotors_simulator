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

#include "rotors_hil_interface/hil_control_interface.h"
//#include "rotors_simulator/rotors_hil_interface/include/rotors_hil_interface/hil_control_interface.h"

namespace rotors_hil {

HilControlInterface::HilControlInterface():
  base_mode_(0),
  system_status_(255)
{
  ros::NodeHandle pnh("~");

  std::string mavlink_sub_topic;
  std::string mav_mode_pub_topic;
  std::string mav_status_pub_topic;
  std::string actuators_pub_topic;
  pnh.param("mavlink_sub_topic", mavlink_sub_topic, kDefaultMavlinkSubTopic);
  pnh.param("mav_mode_pub_topic", mav_mode_pub_topic, kDefaultMavModePubTopic);
  pnh.param("mav_status_pub_topic", mav_status_pub_topic, kDefaultMavStatusPubTopic);
  pnh.param("actuators_pub_topic", actuators_pub_topic, kDefaultActuatorsPubTopic);
  pnh.param("roll_min", roll_min_, kDefaultRollMin);
  pnh.param("roll_max", roll_max_, kDefaultRollMax);
  pnh.param("pitch_min", pitch_min_, kDefaultPitchMin);
  pnh.param("pitch_max", pitch_max_, kDefaultPitchMax);
  pnh.param("yaw_min", yaw_min_, kDefaultYawMin);
  pnh.param("yaw_max", yaw_max_, kDefaultYawMax);

  mavlink_sub_ = nh_.subscribe(mavlink_sub_topic, 15, &HilControlInterface::MavlinkCallback, this);

  mav_mode_pub_ = nh_.advertise<std_msgs::UInt8>(mav_mode_pub_topic, 1);
  mav_status_pub_ = nh_.advertise<std_msgs::UInt8>(mav_status_pub_topic, 1);
  actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(actuators_pub_topic, 1);
}

HilControlInterface::~HilControlInterface() {
}

void HilControlInterface::MavlinkCallback(const mavros_msgs::MavlinkConstPtr& mavros_msg) {
  if (mavros_msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    mavlink_message_t* mavlink_msg(new mavlink_message_t);
    mavros_msgs::mavlink::convert(*mavros_msg, *mavlink_msg);

    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(mavlink_msg, &heartbeat);

    delete mavlink_msg;

    if (base_mode_ != heartbeat.base_mode) {
      base_mode_ = heartbeat.base_mode;

      std_msgs::UInt8 mav_mode_msg;
      mav_mode_msg.data = base_mode_;
      mav_mode_pub_.publish(mav_mode_msg);
    }

    if (system_status_ != heartbeat.system_status) {
      system_status_ = heartbeat.system_status;

      std_msgs::UInt8 mav_status_msg;
      mav_status_msg.data = system_status_;
      mav_status_pub_.publish(mav_status_msg);
    }
  }
  else if (mavros_msg->msgid == MAVLINK_MSG_ID_HIL_CONTROLS) {
    mavlink_message_t* mavlink_msg(new mavlink_message_t);
    mavros_msgs::mavlink::convert(*mavros_msg, *mavlink_msg);

    mavlink_hil_controls_t hil_controls;
    mavlink_msg_hil_controls_decode(mavlink_msg, &hil_controls);

    delete mavlink_msg;

    mav_msgs::Actuators act_msg;

    ros::Time current_time = ros::Time::now();

    //float aileron_angle = (roll_max_ + roll_min_) * 0.5 + (roll_max_ - roll_min_) * 0.5 * hil_controls.roll_ailerons;
    //float elevator_angle = (pitch_max_ + pitch_min_) * 0.5 + (pitch_max_ - pitch_min_) * 0.5 * hil_controls.pitch_elevator;
    //float rudder_angle = (yaw_max_ + yaw_min_) * 0.5 + (yaw_max_ - yaw_min_) * 0.5 * hil_controls.yaw_rudder;
    //float throttle = hil_controls.throttle * 2.0 - 1.0;

    //act_msg.angles.push_back(aileron_angle);
    //act_msg.angles.push_back(elevator_angle);
    //act_msg.angles.push_back(rudder_angle);
    //act_msg.normalized.push_back(throttle);

    //
    // TODO ... make a parameter
    //
    float max_rotor_velocity = 838.0f;
    act_msg.angular_velocities.push_back(hil_controls.throttle * max_rotor_velocity);

    act_msg.angles.push_back(hil_controls.roll_ailerons);
    act_msg.angles.push_back(hil_controls.pitch_elevator);
    act_msg.angles.push_back(hil_controls.yaw_rudder);

    act_msg.header.seq = 1;
    act_msg.header.stamp.sec = current_time.sec;
    act_msg.header.stamp.nsec = current_time.nsec;

    actuators_pub_.publish(act_msg);
  }
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_hil_control_interface");
  rotors_hil::HilControlInterface hil_control_interface;

  ros::spin();

  return 0;
}
