#!/usr/bin/env python
import os
import rospy
import rospkg
import numpy as np
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import SetMode
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QFormLayout
from python_qt_binding.QtCore import QTimer, Slot
from std_msgs.msg import Bool

class MavModeWidget(QWidget):
  # MAV mode flags
  MAV_MODE_FLAG_SAFETY_ARMED = 128
  MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
  MAV_MODE_FLAG_HIL_ENABLED = 32
  MAV_MODE_FLAG_STABILIZE_ENABLED = 16
  MAV_MODE_FLAG_GUIDED_ENABLED = 8
  MAV_MODE_FLAG_AUTO_ENABLED = 4
  MAV_MODE_FLAG_TEST_ENABLED = 2
  MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

  # MAV state dictionary
  mav_state = {0: 'Uninitialized',
  1: 'Booting up',
  2: 'Calibrating',
  3: 'Standby',
  4: 'Active',
  5: 'Critical',
  6: 'Emergency',
  7: 'Poweroff'}

  # String constants
  STR_UNKNOWN = 'N/A'
  STR_SYS_STATUS_SUB_TOPIC = '/mavros/state'
  STR_RESET_MODEL_PUB_TOPIC = 'reset_model'
  STR_START_RECONSTRUCTION_PUB_TOPIC = 'start_reconstruction'

  def __init__(self, parent):
    # Init QWidget
    super(MavModeWidget, self).__init__(parent)
    self.setObjectName('MavModeWidget')
    
    # Load UI
    ui_file = os.path.join(rospkg.RosPack().get_path('rqt_rotors'), 'resource', 'mav_mode_widget.ui')
    loadUi(ui_file, self)

    # Set the initial parameters of UI elements
    self.button_set_hil_mode.setEnabled(False)
    self.button_arm.setEnabled(False)
    self.button_reboot_autopilot.setEnabled(False)
    self.text_state.setText(self.STR_UNKNOWN)
    self.clear_mav_mode()

    # Initialize class variables
    self.mav_mode = 65
    self.mav_status = 255
    self.received_heartbeat = False
    self.armed = False
    self.guided = False
    self.reconstruction_started = False
    
    # Set the functions that are called when signals are emitted
    self.button_set_hil_mode.pressed.connect(self.on_set_hil_mode_button_pressed)
    self.button_arm.pressed.connect(self.on_arm_button_pressed)
    self.button_reboot_autopilot.pressed.connect(self.on_reboot_autopilot_button_pressed)
    self.button_reset_model.pressed.connect(self.on_reset_model_button_pressed)
    self.button_reconstruct.pressed.connect(self.on_reconstruct_button_pressed)

    # Create ROS service proxies
    self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    self.send_command_long = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
    self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    # Initialize ROS subscribers and publishers
    self.sys_status_sub = rospy.Subscriber(self.STR_SYS_STATUS_SUB_TOPIC, State, self.sys_status_callback, queue_size=1)
    self.reset_model_pub = rospy.Publisher(self.STR_RESET_MODEL_PUB_TOPIC, Bool, queue_size=1)
    self.start_reconstruction_pub = rospy.Publisher(self.STR_START_RECONSTRUCTION_PUB_TOPIC, Bool, queue_size=1)

  def on_set_hil_mode_button_pressed(self):
    new_mode = self.mav_mode | self.MAV_MODE_FLAG_HIL_ENABLED
    self.set_mode(new_mode, '')

  def on_arm_button_pressed(self):
    self.arm(True)

  def on_reboot_autopilot_button_pressed(self):
    self.send_command_long(False, 246, 1, 1, 0, 0, 0, 0, 0, 0)

  def on_reset_model_button_pressed(self):
    self.reset_model_pub.publish(True)

  def on_reconstruct_button_pressed(self):
    if not(self.reconstruction_started):
      self.start_reconstruction_pub.publish(True)
    self.reconstruction_started = True
    
  def sys_status_callback(self, msg):
    if not(self.received_heartbeat):
      self.button_set_hil_mode.setEnabled(True)
      self.button_arm.setEnabled(True)
      self.button_reboot_autopilot.setEnabled(True)
      self.received_heartbeat = True

      self.text_mode_safety_armed.setText(self.mav_mode_text(msg.armed))
      self.text_mode_guided.setText(self.mav_mode_text(msg.guided))

    if (self.armed != msg.armed):
      self.armed = msg.armed
      self.text_mode_safety_armed.setText(self.mav_mode_text(self.armed))
      self.button_arm.setEnabled(not(self.armed))

    if (self.guided != msg.guided):
      self.guided = msg.guided
      self.text_mode_guided.setText(self.mav_mode_text(self.guided))

  def clear_mav_mode(self):
    count = self.mav_mode_layout.rowCount()
    for i in range(count):
      self.mav_mode_layout.itemAt(i, QFormLayout.FieldRole).widget().setText(self.STR_UNKNOWN)

  # def process_mav_mode(self, mode):
  #   self.text_mode_safety_armed.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_SAFETY_ARMED))
  #   self.text_mode_manual_input.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED))
  #   self.text_mode_hil.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_HIL_ENABLED))
  #   self.text_mode_stabilize.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_STABILIZE_ENABLED))
  #   self.text_mode_guided.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_GUIDED_ENABLED))
  #   self.text_mode_auto.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_AUTO_ENABLED))
  #   self.text_mode_test.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_TEST_ENABLED))
  #   self.text_mode_custom_mode.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))

  def mav_mode_text(self, mode_enabled):
    return 'ON' if mode_enabled else 'OFF'
