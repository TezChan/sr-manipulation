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
from handle_demo_step_actions.msg import ObjectSelectionActionFeedback
from handle_demo_step_actions.srv import ObjectSelectionFromList, ObjectSelectionFromListResponse


class ObjectSelectionControl(SequenceStep):
    """Class to control the object selection step"""
    def __init__(self, sequence, config):
        super(ObjectSelectionControl, self).__init__(sequence, config)

        self_dir        = os.path.dirname(os.path.realpath(__file__))
        self.ui_dir     = os.path.join(self_dir, '../../ui')

        # info bar UI setup
        self.info_bar_widget.step_name_label.setText(self.config_dict['label'])

        # info area UI setup
        self.info_area_widget = QWidget()
        ui_file = os.path.join(self.ui_dir, 'ObjectSelectionInfoArea.ui')
        loadUi(ui_file, self.info_area_widget)

        self.init_step_info_area()

        self.connect(self,
                     SIGNAL('objectSelection(PyQt_PyObject)'),
                     self.on_object_selection_request)

        self.feedback_subscription()

        #Setup a server to let the object_selection actionlib server ask for the object to select from a list
        srvname = 'decision_services/object_selection'
        self.object_selection_server = rospy.Service(srvname, ObjectSelectionFromList, self.object_selection_callback)

    def feedback_subscription(self):
        self.feedback_listener = rospy.Subscriber('select_object/feedback', ObjectSelectionActionFeedback, self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        self.set_progress_bar_value(feedback_msg.feedback.percent_complete)
        self.print_general_info_area_log(feedback_msg.feedback.feedback_information_message)

    def object_selection_callback(self, req):
        self.selected_object_index = None
        self.emit(SIGNAL('objectSelection(PyQt_PyObject)'), req.objects)

        while not rospy.is_shutdown():
            if self.selected_object_index != None:
                break
            rospy.sleep(0.1)

        return ObjectSelectionFromListResponse(self.selected_object_index)

    def on_object_selection_request(self, objects):
        self.info_area_widget.treeWidget.clear()
        first_item = None

        for (i, object) in enumerate(objects):
            item = QTreeWidgetItem(self.info_area_widget.treeWidget)

            item.setText(0, str(i))
            obj = object.model_description
            item.setText(1, obj.name)

            if "unknown_" not in obj.name:
                item.setText(2, obj.maker)

                tags = ""
                for tag in obj.tags:
                    tags += str(tag) + " ; "
                item.setText(3, tags)

            self.info_area_widget.treeWidget.resizeColumnToContents(0)
            self.info_area_widget.treeWidget.resizeColumnToContents(1)
            self.info_area_widget.treeWidget.resizeColumnToContents(2)
            self.info_area_widget.treeWidget.resizeColumnToContents(3)

        self.info_area_widget.usr_msg_label.setText("Doubleclick the object to be picked from the following list")
        btn_pressed = QMessageBox.information(self.info_area_widget, "User action required", "Doubleclick the object to be picked from the list", buttons = QMessageBox.Ok )
        #Here we only use the messagebox as a way to warn the user, but we could have a messagebox with more than one button, e.g.
        #btn_pressed = QMessageBox.information(self.info_area_widget, "User action required", "Doubleclick the object to be picked from the list", buttons = QMessageBox.Ok |  QMessageBox.Cancel)
        #and then we could use btn_pressed to take a decision

    def init_step_info_area(self):
        self.connect(self.info_area_widget.treeWidget,
                     SIGNAL('itemDoubleClicked (QTreeWidgetItem *, int)'),
                     self.double_click)
        self.info_area_widget.treeWidget.setHeaderLabels(["Index", "Object Name", "Maker", "tags"])
        self.info_area_widget.treeWidget.resizeColumnToContents(0)
        self.info_area_widget.treeWidget.resizeColumnToContents(1)
        self.info_area_widget.treeWidget.resizeColumnToContents(2)
        self.info_area_widget.treeWidget.resizeColumnToContents(3)

    def double_click(self, item, value):
        self.selected_object_index = int(str(item.text(0)))


