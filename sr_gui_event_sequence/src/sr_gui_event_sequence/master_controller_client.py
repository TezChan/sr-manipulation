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

import roslib; roslib.load_manifest('sr_gui_event_sequence')
import rospy
import actionlib
from rospy import loginfo, logerr, logdebug

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import QtGui
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint, SIGNAL
from QtGui import *

from handle_demo_step_actions.msg import MasterControllerAction, MasterControllerGoal
from handle_demo_step_actions.srv import NextStepSelection, NextStepSelectionResponse


class MasterControllerClient(QObject):
    """The base class for all the sequence steps"""
    def __init__(self, step_name_list):
        QObject.__init__(self)

        self.step_name_list = step_name_list
        #Will be filled by the name of the next step selected by the user
        #Or the word "next". Before every user selection it will be set to None
        self.next_step_selected = None

        server_name = 'master_controller'
        self.client = actionlib.SimpleActionClient(server_name, MasterControllerAction)

        #Setup a server to let the MC ask for the next step to run
        srvname = 'decision_services/next_step'
        self.next_step_server = rospy.Service(srvname, NextStepSelection, self.next_step_selection_callback)

    def on_cancel_button_pressed(self):
        self.client.cancel_goal()

    def on_start_step_signal(self, first_step):
        goal = MasterControllerGoal()
        goal.steps_to_execute = self.step_name_list
        goal.first_step = first_step

        self.client.send_goal(goal, done_cb = self.goal_done_callback, feedback_cb = self.action_server_feedback_callback)

    def goal_done_callback(self, terminal_state, result):
        if result.result == 0:
            self.emit(SIGNAL('goalDone(PyQt_PyObject)'), "MC finished succesfully")
        else:
            self.emit(SIGNAL('goalDone(PyQt_PyObject)'), "MC finished without success!!!")

    def action_server_feedback_callback(self, feedback_msg):
        self.emit(SIGNAL('currentStepReceived(PyQt_PyObject)'), feedback_msg.current_step)

    def next_step_selection_callback(self, req):
        self.next_step_selected = None
        self.emit(SIGNAL('nextStepSelection()'))

        while not rospy.is_shutdown():
            if self.next_step_selected:
                break
            rospy.sleep(0.1)

        return NextStepSelectionResponse(self.next_step_selected)


