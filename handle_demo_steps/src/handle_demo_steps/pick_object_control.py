# Copyright 2012 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

# Author: Toni Oliver

import os

import roslib; roslib.load_manifest('handle_demo_steps')
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import QtGui
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint, SIGNAL
from QtGui import *

from sr_gui_event_sequence.sequence_step import SequenceStep
from handle_demo_step_actions.msg import PickObjectActionFeedback


class PickObjectControl(SequenceStep):
    """Class to control the object picking step"""
    def __init__(self, sequence, config):
        super(PickObjectControl, self).__init__(sequence, config)

        self_dir        = os.path.dirname(os.path.realpath(__file__))
        self.ui_dir     = os.path.join(self_dir, '../../ui')

        # info bar UI setup
        self.info_bar_widget.step_name_label.setText(self.config_dict['label'])

        # info area UI setup
        self.info_area_widget = QWidget()
        ui_file = os.path.join(self.ui_dir, 'PickObjectInfoArea.ui')
        loadUi(ui_file, self.info_area_widget)

        self.feedback_subscription()


    def feedback_subscription(self):
        self.feedback_listener = rospy.Subscriber('pick_object/feedback', PickObjectActionFeedback, self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        self.set_progress_bar_value(feedback_msg.feedback.percent_complete)
        self.print_general_info_area_log(feedback_msg.feedback.feedback_information_message)
