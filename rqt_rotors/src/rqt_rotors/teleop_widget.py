#!/usr/bin/env python
import os
import rospkg
import select
import sys
import termios
import tty
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QFormLayout
from python_qt_binding.QtCore import QTimer, Slot

class TeleopWidget(QWidget):
  def __init__(self, parent):
    # Init QWidget
    super(TeleopWidget, self).__init__(parent)
    self.setObjectName('TeleopWidget')
    
    # Load UI
    ui_file = os.path.join(rospkg.RosPack().get_path('rqt_rotors'), 'resource', 'teleop_widget.ui')
    loadUi(ui_file, self)
    
    # Set the functions that are called when signals are emitted
    self.group_box_teleop_key.clicked.connect(self.on_teleop_key_group_clicked)

  def on_teleop_key_group_clicked(self):
    is_teleop_key_enabled = self.group_box_teleop_key.isChecked()
    self.slider_throttle.setEnabled(is_teleop_key_enabled)
    self.slider_rudder.setEnabled(is_teleop_key_enabled)
    self.slider_elevator.setEnabled(is_teleop_key_enabled)
    self.slider_aileron.setEnabled(is_teleop_key_enabled)