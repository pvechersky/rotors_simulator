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

class HilWidget(QWidget):
  # MAV mode flags
  MAV_MODE_FLAG_SAFETY_ARMED = 128
  MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
  MAV_MODE_FLAG_HIL_ENABLED = 32
  MAV_MODE_FLAG_STABILIZE_ENABLED = 16
  MAV_MODE_FLAG_GUIDED_ENABLED = 8
  MAV_MODE_FLAG_AUTO_ENABLED = 4
  MAV_MODE_FLAG_TEST_ENABLED = 2
  MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

  # String constants
  STR_UNKNOWN = 'N/A'
  STR_MAV_MODE_SUB_TOPIC = 'mav_mode'
  STR_MAV_STATUS_SUB_TOPIC = 'mav_status'
  STR_HIL_MODE_PUB_TOPIC = 'hil_mode'

  def __init__(self, rotors_namespace='firefly'):
    # Init QWidget
    super(HilWidget, self).__init__()
    self.setObjectName('HilWidget')
    
    # Load UI
    ui_file = os.path.join(rospkg.RosPack().get_path('rqt_rotors'), 'resource', 'hil_widget.ui')
    loadUi(ui_file, self)

    # Set the initial parameters of UI elements
    self.button_set_hil_mode.setEnabled(False)
    self.text_mav_status.setText(self.STR_UNKNOWN)
    self.clearMavMode()

    # Initialize class variables
    self.mav_mode = 0
    self.mav_status = 255
    self.hil_enabled = False
    
    # Set the functions that are called when a button is pressed
    self.button_set_hil_mode.pressed.connect(self.on_set_hil_mode_button_pressed)
    
    # Initialize ROS subscribers and publishers
    self.mav_mode_sub = rospy.Subscriber(self.STR_MAV_MODE_SUB_TOPIC, UInt8, self.mavModeCallback, queue_size=1)
    self.mav_status_sub = rospy.Subscriber(self.STR_MAV_STATUS_SUB_TOPIC, UInt8, self.mavStatusCallback, queue_size=1)
    self.hil_mode_pub = rospy.Publisher(self.STR_HIL_MODE_PUB_TOPIC, Bool, queue_size=1)

  def on_set_hil_mode_button_pressed(self):
    self.hil_mode_pub.publish(not(self.hil_enabled))
    
  def mavModeCallback(self, msg):
    if not(self.button_set_hil_mode.isEnabled()):
      self.button_set_hil_mode.setEnabled(True)

    if (self.mav_mode != msg.data):
      self.mav_mode = msg.data
      self.processMavMode(msg.data)

      self.hil_enabled = (msg.data & self.MAV_MODE_FLAG_HIL_ENABLED)

      if (self.hil_enabled):
        self.button_set_hil_mode.setText('Disable HIL')
      else:
        self.button_set_hil_mode.setText('Enable HIL')

  def mavStatusCallback(self, msg):
    if (self.mav_status != msg.data):
      self.mav_status = msg.data
      self.text_mav_status.setText(str(msg.data))

  def clearMavMode(self):
    count = self.mav_mode_layout.rowCount()
    for i in range(count):
      self.mav_mode_layout.itemAt(i, QFormLayout.FieldRole).widget().setText(self.STR_UNKNOWN)

  def processMavMode(self, mode):
    self.text_mode_safety_armed.setText(self.mavModeText(mode, self.MAV_MODE_FLAG_SAFETY_ARMED))
    self.text_mode_manual_input.setText(self.mavModeText(mode, self.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED))
    self.text_mode_hil.setText(self.mavModeText(mode, self.MAV_MODE_FLAG_HIL_ENABLED))
    self.text_mode_stabilize.setText(self.mavModeText(mode, self.MAV_MODE_FLAG_STABILIZE_ENABLED))
    self.text_mode_guided.setText(self.mavModeText(mode, self.MAV_MODE_FLAG_GUIDED_ENABLED))
    self.text_mode_auto.setText(self.mavModeText(mode, self.MAV_MODE_FLAG_AUTO_ENABLED))
    self.text_mode_test.setText(self.mavModeText(mode, self.MAV_MODE_FLAG_TEST_ENABLED))
    self.text_mode_custom_mode.setText(self.mavModeText(mode, self.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED))

  def mavModeText(self, mode, flag):
    return 'ON' if (mode & flag) else 'OFF'
