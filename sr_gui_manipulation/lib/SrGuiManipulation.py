# Copyright 2011 Shadow Robot Company Ltd.
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

from __future__ import division
import os

import roslib; roslib.load_manifest('sr_gui_manipulation')
import rospy
from rospy import loginfo, logerr, logdebug

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint
from QtGui import QDockWidget, QShortcut, QMessageBox, QFrame, QHBoxLayout, QCheckBox, QLabel, QCursor, QColor, QMessageBox

from tabletop_object_detector.srv import TabletopDetection
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from household_objects_database_msgs.srv import GetModelDescription

class TableObject(object):
    """
    Contains all the relevant info for an object.
    A map of TableObject is stored in the ObjectSelection plugin.
    """
    def __init__(self):
        self.graspable_object = None
        self.graspable_object_name = None
        self.model_description = None

class Model(object):
    def __init__(self):
        self.name = None

class SrGuiManipulation(QObject):

    def __init__(self, parent, plugin_context):
        super(SrGuiManipulation, self).__init__(parent)

        self.setObjectName('SrGuiManipulation')
        # Set by call to process_collision_map()
        self.collision_support_surface_name = None
        self.raw_objects            = None
        self.found_objects          = {}
        self.unknown_object_counter = 1 #starts at 1 as it's only used for display
        self.service_tabletop_collision_map   = None
        self.service_db_get_model_description = None
        self.service_object_detector          = None

        # UI setup
        ui_file = os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                '../ui/SrGuiManipulation.ui')

        main_window = plugin_context.main_window()
        self.win = QDockWidget(main_window)
        loadUi(ui_file, self.win)
        if plugin_context.serial_number() > 1:
            self.win.setWindowTitle(
                    self.win.windowTitle() + (' (%d)' % plugin_context.serial_number()) )
        main_window.addDockWidget(Qt.RightDockWidgetArea, self.win)

        # trigger deleteLater for plugin when win is closed
        self.win.installEventFilter(self)

        # Bind button clicks
        self.win.btn_detect_objects.pressed.connect(self.detect_objects)
        self.win.btn_collision_map.pressed.connect(self.process_collision_map)
        self.win.btn_collision_map.setEnabled(False)

        # Service setup
        srvname = '/tabletop_collision_map_processing/tabletop_collision_map_processing'
        rospy.wait_for_service(srvname)
        self.service_tabletop_collision_map = rospy.ServiceProxy(srvname, TabletopCollisionMapProcessing)

        srvname = 'object_detection'
        rospy.wait_for_service(srvname)
        self.service_object_detector = rospy.ServiceProxy(srvname, TabletopDetection)

        srvname = 'objects_database_node/get_model_description'
        rospy.wait_for_service(srvname)
        self.service_db_get_model_description = rospy.ServiceProxy(srvname, GetModelDescription)

    def hello(self):
        """Say hello, useful for quick test of buttons"""
        loginfo("hello")

    def eventFilter(self, obj, event):
        if obj is self.win and event.type() == QEvent.Close:
            # TODO: ignore() should not be necessary when returning True
            event.ignore()
            self.deleteLater()
            return True
        return QObject.eventFilter(self, obj, event)

    def close_plugin(self):
        self.win.close()
        self.win.deleteLater()

    def save_settings(self, global_settings, perspective_settings):
        loginfo(self.objectName()+" saving settings")

    def restore_settings(self, global_settings, perspective_settings):
        loginfo(self.objectName()+" restoring settings")

    def detect_objects(self):
        self.found_objects.clear()
        try:
            self.raw_objects = self.service_object_detector(True, True, 1)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)

        # Take a new collision map + add the detected objects to the collision
        # map and get graspable objects from them
        tabletop_collision_map_res = self.process_collision_map()
        if tabletop_collision_map_res != 0:
           for grasp_obj, grasp_obj_name in zip(tabletop_collision_map_res.graspable_objects, tabletop_collision_map_res.collision_object_names):
               #loginfo("Found: {0} {1}".format(grasp_obj,grasp_obj_name))
               loginfo("Found: %s" % grasp_obj_name)
               obj_tmp = TableObject()
               obj_tmp.graspable_object = grasp_obj
               obj_tmp.graspable_object_name = grasp_obj_name

               if len(grasp_obj.potential_models) > 0:
                   model_index = grasp_obj.potential_models[0].model_id
               else:
                   model_index = -1
               obj_tmp.model_description = self.get_object_name(model_index)

               self.found_objects[obj_tmp.model_description.name] = obj_tmp

        if self.raw_objects != None:
           self.win.btn_collision_map.setEnabled(True)
        #self.parent.parent.reload_object_signal_widget.reloadObjectSig['int'].emit(1)

    def process_collision_map(self):
        res = 0
        try:
            #Args: detection_result reset_collision_models reset_attached_models desired_frame
            res = self.service_tabletop_collision_map.call(self.raw_objects.detection, True, True, "/fixed")
        except rospy.ServiceException, e:
            logerr("Service did not process request: %s" % str(e))

        if res != 0:
            loginfo("collision_support_surface_name: "+res.collision_support_surface_name)
            self.collision_support_surface_name = res.collision_support_surface_name
        return res

    def get_object_name(self, model_id):
        """
        return the object name given its index (read from database, or
        create unique name if unknown object).
        """
        model = Model()
        #todo make sure name is unique
        if model_id == -1:
            model.name = "unknown_" + str(self.unknown_object_counter)
            self.unknown_object_counter += 1
        else:
            try:
                model = self.service_db_get_model_description(model_id)
            except rospy.ServiceException, e:
                print "Service did not process request: %s" % str(e)
                model.name = "unkown_recognition_failed"
        return model

