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

from qt_gui.qt_binding_helper import loadUi
import QtGui
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint, SIGNAL
from QtGui import *


class SequenceStep(QObject):
    """The base class for all the sequence steps"""
    def __init__(self, sequence, config):
        QObject.__init__(self)
        #Will contain the results of the execution of this step
        self.results = None
        #Will contain the commands given by the user. This will be considered by subsequent steps
        self.commands = None
        #Dictionary of all the steps. This allows this object to have access to the info (results, user commands) from the other steps
        self.sequence_dict = sequence
        #Dictionary containing the configuration (name, tag, etc) for this step. This info comes from the yaml file
        self.config_dict = config
        #Information bar for this step shown in the sequence list. This will be loaded by a subclass
        self.info_bar_widget = None
        #Information area for this step shown when this step is active or selected. This will be loaded by a subclass
        self.info_area_widget = None
        
        #Listener for the feedback topic of this step's actionlib
        self.feedback_listener = None

        self_dir        = os.path.dirname(os.path.realpath(__file__))
        self.bar_ui_dir     = os.path.join(self_dir, '../../ui')
        self.ui_dir     = None
        
        # info bar UI setup
        self.info_bar_widget = QFrame()
        ui_file = os.path.join(self.bar_ui_dir, 'StepBar.ui')
        loadUi(ui_file, self.info_bar_widget)
        #Connect info bar events 
        self.info_bar_widget.show_checkBox.stateChanged.connect(self.on_show_checkbox_state_changed)
        self.connect(self,
                     SIGNAL('changeProgressBarValue(int)'),
                     self.on_change_progress_bar_value)
        
        #Run and start buttons in the bar will be initially disabled as the first time we run the sequence
        #it only makes sense to start from the beginning
        self.info_bar_widget.btn_start.setEnabled(False)
        self.info_bar_widget.btn_run.setEnabled(False)
        self.info_bar_widget.btn_start.pressed.connect(self.on_btn_start_pressed)
        self.info_bar_widget.btn_run.pressed.connect(self.on_btn_run_pressed)
        



    def update_info_area_widget(self):
        raise NotImplementedError, "Virtual method, please implement."
    
    
    
    def feedback_subscription(self):
        raise NotImplementedError, "Virtual method, please implement."
    
    def feedback_callback(self, feedback_msg):
        raise NotImplementedError, "Virtual method, please implement."
    
    def set_progress_bar_value(self, progress):
        self.emit(SIGNAL('changeProgressBarValue(int)'), progress)
        
    def on_change_progress_bar_value(self, progress):
        self.info_bar_widget.progressBar.setValue(progress)
        
    def on_show_checkbox_state_changed(self, state):
        self.emit(SIGNAL('showStateChanged(int, PyQt_PyObject)'), state, self.config_dict['name'])
    
    def on_btn_start_pressed(self):
        self.emit(SIGNAL('startStepPressed(PyQt_PyObject)'), self.config_dict['name'])
        
    def on_btn_run_pressed(self):
        self.emit(SIGNAL('runStepPressed(PyQt_PyObject)'), self.config_dict['name'])
    
    def on_show_run_btn_signal(self, state):
        self.info_bar_widget.btn_run.setEnabled(state)
        
    def on_show_start_btn_signal(self, state):
        self.info_bar_widget.btn_start.setEnabled(state)

    def print_general_info_area_log(self, log_msg):
        self.emit(SIGNAL('printGeneralInfoAreaLog(PyQt_PyObject, PyQt_PyObject)'), self.config_dict['name'], log_msg)

