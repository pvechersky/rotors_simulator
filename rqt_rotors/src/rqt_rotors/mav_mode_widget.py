#!/usr/bin/env python
import os
import rospy
import rospkg
import numpy as np
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QFormLayout
from python_qt_binding.QtCore import QTimer, Slot
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

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
  STR_MAV_MODE_SUB_TOPIC = 'mav_mode'
  STR_MAV_STATUS_SUB_TOPIC = 'mav_status'
  STR_SET_MODE_PUB_TOPIC = 'set_mode'
  STR_RESET_MODEL_PUB_TOPIC = 'reset'

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
    self.text_state.setText(self.STR_UNKNOWN)
    self.clear_mav_mode()

    # Initialize class variables
    self.mav_mode = 0
    self.mav_status = 255
    self.received_heartbeat = False
    self.hil_enabled = False
    self.armed = False
    
    # Set the functions that are called when signals are emitted
    self.button_set_hil_mode.pressed.connect(self.on_set_hil_mode_button_pressed)
    self.button_arm.pressed.connect(self.on_arm_button_pressed)
    self.button_reset_model.pressed.connect(self.on_reset_model_button_pressed)
    
    # Initialize ROS subscribers and publishers
    self.mav_mode_sub = rospy.Subscriber(self.STR_MAV_MODE_SUB_TOPIC, UInt8, self.mav_mode_callback, queue_size=1)
    self.mav_status_sub = rospy.Subscriber(self.STR_MAV_STATUS_SUB_TOPIC, UInt8, self.mav_status_callback, queue_size=1)
    self.set_mode_pub = rospy.Publisher(self.STR_SET_MODE_PUB_TOPIC, UInt8, queue_size=1)
    self.reset_pub = rospy.Publisher(self.STR_RESET_MODEL_PUB_TOPIC, Bool, queue_size=1)

  def on_set_hil_mode_button_pressed(self):
    new_mode = self.mav_mode | self.MAV_MODE_FLAG_HIL_ENABLED
    self.set_mode_pub.publish(new_mode)

  def on_arm_button_pressed(self):
    new_mode = self.mav_mode | self.MAV_MODE_FLAG_SAFETY_ARMED
    self.set_mode_pub.publish(new_mode)

  def on_reset_model_button_pressed(self):
    self.reset_pub.publish(True)
    
  def mav_mode_callback(self, msg):
    if not(self.received_heartbeat):
      self.button_set_hil_mode.setEnabled(True)
      self.button_arm.setEnabled(True)
      self.received_heartbeat = True

    if (self.mav_mode != msg.data):
      self.mav_mode = msg.data
      self.process_mav_mode(msg.data)

      new_hil_enabled = (msg.data & self.MAV_MODE_FLAG_HIL_ENABLED)
      new_armed = (msg.data & self.MAV_MODE_FLAG_SAFETY_ARMED)

      #if (self.hil_enabled != new_hil_enabled):
      #  self.hil_enabled = new_hil_enabled
      #  self.button_set_hil_mode.setEnabled(not(new_hil_enabled))

      if (not self.hil_enabled and new_hil_enabled):
        self.hil_enabled = True
        self.button_set_hil_mode.setEnabled(False)

      if (self.hil_enabled and not new_hil_enabled):
        new_mode = self.mav_mode | self.MAV_MODE_FLAG_HIL_ENABLED
        self.set_mode_pub.publish(new_mode)
      
      if (self.armed != new_armed):
        self.armed = new_armed
        self.button_arm.setEnabled(not(new_armed))

  def mav_status_callback(self, msg):
    if (self.mav_status != msg.data):
      self.mav_status = msg.data
      self.text_state.setText(self.mav_state[msg.data])

  def clear_mav_mode(self):
    count = self.mav_mode_layout.rowCount()
    for i in range(count):
      self.mav_mode_layout.itemAt(i, QFormLayout.FieldRole).widget().setText(self.STR_UNKNOWN)

  def process_mav_mode(self, mode):
    self.text_mode_safety_armed.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_SAFETY_ARMED))
    self.text_mode_manual_input.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED))
    self.text_mode_hil.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_HIL_ENABLED))
    self.text_mode_stabilize.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_STABILIZE_ENABLED))
    self.text_mode_guided.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_GUIDED_ENABLED))
    self.text_mode_auto.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_AUTO_ENABLED))
    self.text_mode_test.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_TEST_ENABLED))
    self.text_mode_custom_mode.setText(self.mav_mode_text(mode, self.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))

  def mav_mode_text(self, mode, flag):
    return 'ON' if (mode & flag) else 'OFF'
