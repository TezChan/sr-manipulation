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

from qt_gui.qt_binding_helper import loadUi
import QtGui
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint, SIGNAL
from QtGui import *

from sr_gui_event_sequence.sequence_step import SequenceStep
from handle_demo_step_actions.msg import PlacePositionSelectionActionFeedback
from handle_demo_step_actions.srv import PlacePositionSelection, PlacePositionSelectionResponse
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class PlacePositionSelectionControl(SequenceStep):
    """Class to control the object recognition step"""
    def __init__(self, sequence, config):
        super(PlacePositionSelectionControl, self).__init__(sequence, config)
        
        self_dir        = os.path.dirname(os.path.realpath(__file__))
        self.ui_dir     = os.path.join(self_dir, '../../ui')

        # info bar UI setup
        self.info_bar_widget.step_name_label.setText(self.config_dict['label'])
        
        # info area UI setup
        self.info_area_widget = QWidget()
        ui_file = os.path.join(self.ui_dir, 'PlacePositionSelectionInfoArea.ui')
        loadUi(ui_file, self.info_area_widget)
        
        self.init_step_info_area()
        
        self.connect(self,
                     SIGNAL('positionSelection(PyQt_PyObject)'),
                     self.on_position_selection_request)


        self.feedback_subscription()
        
        #Setup a server to let the position_selection actionlib server ask the user for the position to place the object
        srvname = 'decision_services/place_position_selection'
        self.position_selection_server = rospy.Service(srvname, PlacePositionSelection, self.position_selection_callback)


    def feedback_subscription(self):
        self.feedback_listener = rospy.Subscriber('select_place_position/feedback', PlacePositionSelectionActionFeedback, self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        self.set_progress_bar_value(feedback_msg.feedback.percent_complete)
        self.print_general_info_area_log(feedback_msg.feedback.feedback_information_message)
        
    def position_selection_callback(self, req):
        self.selected_pose = None
        self.emit(SIGNAL('positionSelection(PyQt_PyObject)'), req.initial_pose_of_the_object)
        
        while not rospy.is_shutdown():
            if self.selected_pose != None:
                break
            rospy.sleep(0.1)
        
        return PlacePositionSelectionResponse(self.selected_pose)
    
    def on_position_selection_request(self, initial_pose):
        #We initialize the field with the initial pose of the object
        #to give the user a reference to decide the pose where to place it       
        self.info_area_widget.frame_id_Edit.setText(str(initial_pose.header.frame_id))
        self.info_area_widget.x_Edit.setText(str(initial_pose.pose.position.x))
        self.info_area_widget.y_Edit.setText(str(initial_pose.pose.position.y))
        self.info_area_widget.z_Edit.setText(str(initial_pose.pose.position.z))
        self.info_area_widget.x_Quat_Edit.setText(str(initial_pose.pose.orientation.x))
        self.info_area_widget.y_Quat_Edit.setText(str(initial_pose.pose.orientation.y))
        self.info_area_widget.z_Quat_Edit.setText(str(initial_pose.pose.orientation.z))
        self.info_area_widget.w_Quat_Edit.setText(str(initial_pose.pose.orientation.w))
        self.info_area_widget.usr_msg_label.setText("This was the object initial pose. Please edit it to choose the pose to place it and then click the button.")
        btn_pressed = QMessageBox.information(self.info_area_widget, "User action required", "This was the object initial pose. Please edit it to choose the pose to place it and then click the button.", buttons = QMessageBox.Ok )
        #Here we only use the messagebox as a way to warn the user, but we could have a messagebox with more than one button, e.g.
        #btn_pressed = QMessageBox.information(self.info_area_widget, "User action required", "Doubleclick the object to be picked from the list", buttons = QMessageBox.Ok |  QMessageBox.Cancel)
        #and then we could use btn_pressed to take a decision
        self.info_area_widget.btn_select_place_pose.setEnabled(True)
        
    def init_step_info_area(self):
        self.info_area_widget.btn_select_place_pose.pressed.connect(self.on_select_place_pose_btn_pressed)
     
    def on_select_place_pose_btn_pressed(self):
        pose = PoseStamped()
        pose.header.frame_id = self.info_area_widget.frame_id_Edit.text()
        
        pose.pose.position.x = float(self.info_area_widget.x_Edit.text())
        pose.pose.position.y = float(self.info_area_widget.y_Edit.text())
        pose.pose.position.z = float(self.info_area_widget.z_Edit.text())
        
        pose.pose.orientation.x = float(self.info_area_widget.x_Quat_Edit.text())
        pose.pose.orientation.y = float(self.info_area_widget.y_Quat_Edit.text())
        pose.pose.orientation.z = float(self.info_area_widget.z_Quat_Edit.text())
        pose.pose.orientation.w = float(self.info_area_widget.w_Quat_Edit.text())
        
        self.selected_pose = pose
        self.info_area_widget.btn_select_place_pose.setEnabled(False)
