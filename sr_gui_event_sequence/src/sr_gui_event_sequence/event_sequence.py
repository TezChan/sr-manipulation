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
import math

import scipy
import scipy.linalg
import numpy as np

import roslib; roslib.load_manifest('sr_gui_event_sequence')
import rospy
from rospy import loginfo, logerr, logdebug

'''
from std_msgs.msg import Float64
from etherCAT_hand_lib import EtherCAT_Hand_Lib
from shadowhand_ros import ShadowHand_ROS
'''

from qt_gui.plugin import Plugin
from qt_gui.qt_binding_helper import loadUi
import QtGui
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint, SIGNAL
from QtGui import *

from sr_gui_event_sequence.master_controller_client import MasterControllerClient
'''
from tabletop_object_detector.srv import TabletopDetection
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from household_objects_database_msgs.srv import GetModelDescription
from household_objects_database_msgs.msg import DatabaseModelPose
import object_manipulator.draw_functions as draw_functions
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from object_manipulation_msgs.msg import Grasp, PickupGoal, PickupAction, PickupResult, PlaceGoal, PlaceAction, ManipulationResult, GraspableObject
from object_manipulator.convert_functions import *
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Pose
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
import control_msgs.msg
import trajectory_msgs.msg
from tf import transformations
import tf

import copy

from sr_pick_and_place.execution import Execution
'''
import yaml

class SrGuiEventSequence(Plugin):
    """The main GUI dock window"""
    def __init__(self, context):
        super(SrGuiEventSequence, self).__init__(context)

        self.setObjectName('SrGuiEventSequence')

        self_dir        = os.path.dirname(os.path.realpath(__file__))
        self.config_dir = os.path.join(self_dir, '../../config')
        self.ui_dir     = os.path.join(self_dir, '../../ui')

        # UI setup
        self._widget = QWidget()
        ui_file = os.path.join(self.ui_dir, 'SrGuiEventSequence.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrGuiEventSequenceUi')
        context.add_widget(self._widget)
        
        #Step related 
        self.current_step = None
        #To indicate that the step 0 hasn't been executed yet
        self.first_time = True
        #this list will be filled using the info in the yaml file
        self.step_names_list = []
        self.step_index_dict = {}
        
        #Dictionary containing the SequenceStep objects for every step in the sequence
        self.sequence_dict = {}
        #Dictionary containing the configuration of the steps read from the yaml file
        self.config_data = None
        #Read steps configuration from yaml file 
        yaml_file = os.path.join(self.config_dir, 'sequence_config.yaml')
        try:
            self.config_data = yaml.load(open(yaml_file))
        except IOError as err:
            QMessageBox.warning(self._widget, "Warning",
                    "Failed to load '"+yaml_file+"': "+str(err))
            raise
        
        #Autorun
        if self._widget.autorun_checkBox.isChecked():
            rospy.set_param('/master_controller/autorun', 1)
        else:
            rospy.set_param('/master_controller/autorun', 0)
        
        #Create the Master controller Client
        for (i, step) in enumerate(self.config_data['steps']):
            self.step_names_list.append(step['name'])
            self.step_index_dict[step['name']] = i
            
        self.master_controller_client = MasterControllerClient(self.step_names_list)
        #connect the signal to notify goal_done (all the steps in the sequence finished somehow)
        self.connect(self.master_controller_client,
                     SIGNAL('goalDone(PyQt_PyObject)'),
                     self.on_MC_goal_done)
        #connect the signal to notify goal_done (all the steps in the sequence finished somehow)
        self.connect(self.master_controller_client,
                     SIGNAL('currentStepReceived(PyQt_PyObject)'),
                     self.on_MC_current_step_received)
        #connect the signal that notifies that the user should choose the next step to be run
        self.connect(self.master_controller_client,
                     SIGNAL('nextStepSelection()'),
                     self.on_MC_select_next_step)
        
        
        
        #Import and create an object of the class specified in the yaml for every step
        for step in self.config_data['steps']:
            roslib.load_manifest(step['package'])
            mod = __import__(step['package']  + '.' + step['module'], fromlist=[step['class']])
            klass = getattr(mod, step['class'])
            self.sequence_dict[step['name']] = klass(self.sequence_dict, step)
            #Load the info bar widget for every step
            self._widget.step_list_layout.addWidget(self.sequence_dict[step['name']].info_bar_widget)
            #connect the signal to show the info for that step
            self.connect(self.sequence_dict[step['name']],
                         SIGNAL('showStateChanged(int, PyQt_PyObject)'),
                         self.on_show_state_changed)
            #connect the signal to start the sequence (using this step)
            self.connect(self.sequence_dict[step['name']],
                         SIGNAL('startStepPressed(PyQt_PyObject)'),
                         self.on_start_sequence_signal)
            #connect the signal to tell the MC that this step will be next 
            self.connect(self.sequence_dict[step['name']],
                         SIGNAL('runStepPressed(PyQt_PyObject)'),
                         self.on_run_this_step_signal)
            #connect the signal to print a log to the general info area
            self.connect(self.sequence_dict[step['name']],
                         SIGNAL('printGeneralInfoAreaLog(PyQt_PyObject, PyQt_PyObject)'),
                         self.on_print_general_info_area_log)
            #connect the signal to tell the bar to make the run button visible
            self.connect(self,
                         SIGNAL('showRunStepButtons(PyQt_PyObject)'),
                         self.sequence_dict[step['name']].on_show_run_btn_signal)
            #connect the signal to tell the bar to make the run button visible
            self.connect(self,
                         SIGNAL('showStartSequenceButtons(PyQt_PyObject)'),
                         self.sequence_dict[step['name']].on_show_start_btn_signal)
            
            
            
            
        #Add a vertical spacer after the bars, to hold them together in the beginning of the vertical layout
        self._widget.step_list_layout.addStretch(0)
        
        self.general_info_area_widget = QWidget()
        ui_file = os.path.join(self.ui_dir, 'GeneralInfoArea.ui')
        loadUi(ui_file, self.general_info_area_widget)
        self._widget.general_info_area.addWidget(self.general_info_area_widget)
        
        #Load info area for the first step (maybe we shouldn't before it's running, but we'll show it for the time being)
        for step in self.config_data['steps']:
            if self.sequence_dict[step['name']].info_area_widget != None:
                self.current_step = step['name']
                self.shown_step_widget = self.sequence_dict[self.current_step].info_area_widget
                self._widget.step_info_area.addWidget(self.shown_step_widget)
                break
                

        # Bind button clicks
        self._widget.btn_start.pressed.connect(self.on_btn_start_pressed)
        self._widget.btn_cancel.pressed.connect(self.on_btn_cancel_pressed)
        self._widget.btn_next.pressed.connect(self.on_btn_next_pressed)
        #Connect autorun check/uncheck events 
        self._widget.autorun_checkBox.stateChanged.connect(self.on_autorun_checkbox_state_changed)
        
        #connect the signal to make the next (run) button visible
        self.connect(self,
                     SIGNAL('showRunStepButtons(PyQt_PyObject)'),
                     self.on_show_next_btn_signal)
        #connect the signal to make the start button visible
        self.connect(self,
                     SIGNAL('showStartSequenceButtons(PyQt_PyObject)'),
                     self.on_show_start_btn_signal)
        
        #We'll emit a signal to hide all the buttons to choose the next step
        self.emit(SIGNAL('showRunStepButtons(PyQt_PyObject)'), False)

    def on_show_state_changed(self, state, step_name):
        if state == Qt.Checked:
            if self.sequence_dict[step_name].info_area_widget != None:
                self.shown_step_widget.hide()
                self._widget.step_info_area.removeWidget(self.shown_step_widget)
                self.shown_step_widget = self.sequence_dict[step_name].info_area_widget
                self._widget.step_info_area.addWidget(self.shown_step_widget)
                self.shown_step_widget.show()
            for step in self.config_data['steps']:
                if step['name'] != step_name and \
                   self.sequence_dict[step['name']].info_bar_widget != None and \
                   self.sequence_dict[step['name']].info_bar_widget.show_checkBox.isChecked():
                    self.sequence_dict[step['name']].info_bar_widget.show_checkBox.setChecked(False)
                    break
        else:
            for step_name in self.sequence_dict.keys():
                if self.sequence_dict[step_name].info_bar_widget.show_checkBox.isChecked():
                    return
            #if all the "show info" checkboxes are unchecked, the current_step info area will be shown
            self.shown_step_widget.hide()
            self._widget.step_info_area.removeWidget(self.shown_step_widget)
            self.shown_step_widget = self.sequence_dict[self.current_step].info_area_widget
            self._widget.step_info_area.addWidget(self.shown_step_widget)
            self.shown_step_widget.show()

    def on_start_sequence_signal(self, step_name):
        self.master_controller_client.on_start_step_signal(step_name)
        #We'll emit a signal to hide all the buttons to start sequence
        self.emit(SIGNAL('showStartSequenceButtons(PyQt_PyObject)'), False)
    
    def on_run_this_step_signal(self, step_name):
        self.master_controller_client.next_step_selected = step_name
        #We'll emit a signal to hide all the buttons to choose the next step
        self.emit(SIGNAL('showRunStepButtons(PyQt_PyObject)'), False)
    
    def on_MC_goal_done(self, msg):
        #Show some info in the general info area
        self.general_info_area_widget.log_area_TextEdit.appendHtml("<font color=\"red\">" + msg + "</font>")
        #We'll emit a signal to show all the buttons to start sequence
        self.emit(SIGNAL('showStartSequenceButtons(PyQt_PyObject)'), True)
        self.first_time = True
    
    def on_MC_current_step_received(self, step_name):
        #Show some info in the general info area
        self.general_info_area_widget.log_area_TextEdit.appendHtml("<font color=\"red\">" + "MC Current step is: " + step_name + "</font>")
        #This affects the behaviour of the "next" button
        self.first_time = False
        
        self.current_step = step_name
        
        #Show the step info area for the current step (by unchecking all of them)
        self.sequence_dict[step_name].info_bar_widget.show_checkBox.setChecked(True)
        for step in self.sequence_dict.keys():
            if self.sequence_dict[step].info_bar_widget.show_checkBox.isChecked():
                self.sequence_dict[step].info_bar_widget.show_checkBox.setChecked(False)

    def on_MC_select_next_step(self):
        #We'll emit a signal to show all the buttons to choose the next step
        self.emit(SIGNAL('showRunStepButtons(PyQt_PyObject)'), True)
        
    def on_show_next_btn_signal(self, state):
        self._widget.btn_next.setEnabled(state)
        
    def on_show_start_btn_signal(self, state):
        self._widget.btn_start.setEnabled(state)
        
    def on_btn_start_pressed(self):
        #If the Start button is pressed we send the first (index = 0) step to the MC
        self.on_start_sequence_signal(self.step_names_list[0])
        
    def on_btn_cancel_pressed(self):
        self.master_controller_client.on_cancel_button_pressed()
        
    def on_btn_next_pressed(self):
        next_step_index = None
        if self.first_time:
            next_step_index = 0
            self.first_time = False
        else:
            next_step_index = self.step_index_dict[self.current_step] + 1
        if next_step_index >= len(self.step_names_list):
            next_step_name = "next"
        else:
            next_step_name = self.step_names_list[next_step_index]
        self.on_run_this_step_signal(next_step_name)
        
    def on_print_general_info_area_log(self, step_name, log_msg):        
        #Show some info in the general info area
        self.general_info_area_widget.log_area_TextEdit.appendPlainText(step_name + ": " + log_msg)
        
    def on_autorun_checkbox_state_changed(self, state):
        if state == Qt.Checked:
            rospy.set_param('/master_controller/autorun', 1)
        else:
            rospy.set_param('/master_controller/autorun', 0)
        
    def eventFilter(self, obj, event):
        if obj is self._widget and event.type() == QEvent.Close:
            # TODO: ignore() should not be necessary when returning True
            event.ignore()
            self.deleteLater()
            return True
        return QObject.eventFilter(self, obj, event)

    def shutdown_plugin(self):
        pass


    def save_settings(self, global_settings, perspective_settings):
        pass

    def restore_settings(self, global_settings, perspective_settings):
        pass




