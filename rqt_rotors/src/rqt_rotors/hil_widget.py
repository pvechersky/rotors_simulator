#!/usr/bin/env python
import os
import rospy
import rospkg
import numpy as np
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import QTimer, Slot
from std_msgs.msg import Bool
from std_msgs.msg import UInt8

class HilWidget(QWidget):
  # Class constants
  MAV_MODE_FLAG_HIL_ENABLED = 32

  def __init__(self, rotors_namespace='firefly'):
    # Init QWidget
    super(HilWidget, self).__init__()
    self.setObjectName('HilWidget')
    
    # Load UI
    ui_file = os.path.join(rospkg.RosPack().get_path('rqt_rotors'), 'resource', 'hil_widget.ui')
    loadUi(ui_file, self)

    # Set the initial parameters of UI elements
    self.button_set_hil_mode.setEnabled(False)
    self.text_mav_mode.setText('N/A')
    self.text_mav_status.setText('N/A')

    # Initialize class variables
    self.mav_mode = 0
    self.mav_status = 255
    self.hil_enabled = False
    
    # Set the functions that are called when a button is pressed
    self.button_set_hil_mode.pressed.connect(self.on_set_hil_mode_button_pressed)
    
    # Initialize ROS subscribers and publishers
    self.mav_mode_sub = rospy.Subscriber('mav_mode', UInt8, self.mavModeCallback, queue_size=1)
    self.mav_status_sub = rospy.Subscriber('mav_status', UInt8, self.mavStatusCallback, queue_size=1)
    self.hil_mode_pub = rospy.Publisher('hil_mode', Bool, queue_size=1)

  def on_set_hil_mode_button_pressed(self):
    self.hil_mode_pub.publish(not(self.hil_enabled))
    
  def mavModeCallback(self, msg):
    if not(self.button_set_hil_mode.isEnabled()):
      self.button_set_hil_mode.setEnabled(True)

    if (self.mav_mode != msg.data):
      self.mav_mode = msg.data
      self.text_mav_mode.setText(str(msg.data))

      self.hil_enabled = (msg.data & self.MAV_MODE_FLAG_HIL_ENABLED)

      if (self.hil_enabled):
        self.button_set_hil_mode.setText('Disable HIL')
      else:
        self.button_set_hil_mode.setText('Enable HIL')

  def mavStatusCallback(self, msg):
    if (self.mav_status != msg.data):
      self.mav_status = msg.data
      self.text_mav_status.setText(str(msg.data))