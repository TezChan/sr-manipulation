#!/usr/bin/env python
#
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


import roslib; roslib.load_manifest('handle_demo_step_action_servers')
import rospy
import actionlib

from tabletop_object_detector.srv import TabletopDetection
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from household_objects_database_msgs.srv import GetModelDescription
from handle_demo_step_actions.msg import ObjectRecognitionAction, ObjectRecognitionResult, ObjectRecognitionFeedback, TableObject, ModelDescription


class ObjectRecognitionServer(object):
    def __init__(self):
        self.collision_support_surface_name = None
        self.raw_objects            = None
        self.found_objects          = []
        self.unknown_object_counter = 1 #starts at 1 as it's only used for display
        self.service_tabletop_collision_map   = None
        self.service_db_get_model_description = None
        self.service_object_detector          = None
        
        self.init_services()
        
        self.server = actionlib.SimpleActionServer('recognise_objects', ObjectRecognitionAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Step actionlib servers: object_recognition server ready")
        
    def init_services(self):
        """Service setup"""
        srvname = '/tabletop_collision_map_processing/tabletop_collision_map_processing'
        rospy.loginfo("Waiting for service " + srvname)
        rospy.wait_for_service(srvname)
        self.service_tabletop_collision_map = rospy.ServiceProxy(srvname, TabletopCollisionMapProcessing)

        srvname = 'object_detection'
        rospy.loginfo("Waiting for service " + srvname)
        rospy.wait_for_service(srvname)
        self.service_object_detector = rospy.ServiceProxy(srvname, TabletopDetection)

        srvname = 'objects_database_node/get_model_description'
        rospy.loginfo("Waiting for service " + srvname)
        rospy.wait_for_service(srvname)
        self.service_db_get_model_description = rospy.ServiceProxy(srvname, GetModelDescription)

    def execute(self, goal):
        self.detect_objects()    
        self.server.set_succeeded(ObjectRecognitionResult(self.found_objects, self.collision_support_surface_name))
    
    def detect_objects(self):
        #Publish some feedback
        self.server.publish_feedback(ObjectRecognitionFeedback(1, "Init object detection"))
        del self.found_objects[:]
        
        try:
            self.raw_objects = self.service_object_detector(True, True, 1)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)

        #Publish some feedback
        self.server.publish_feedback(ObjectRecognitionFeedback(25, "Tabletop detection done"))

        # Take a new collision map + add the detected objects to the collision
        # map and get graspable objects from them
        tabletop_collision_map_res = self.process_collision_map()
        
        #Publish some feedback
        self.server.publish_feedback(ObjectRecognitionFeedback(75, "Collision map processed"))
        
        if tabletop_collision_map_res != 0:
           for grasp_obj, grasp_obj_name in zip(tabletop_collision_map_res.graspable_objects, tabletop_collision_map_res.collision_object_names):
               #loginfo("Found: {0} {1}".format(grasp_obj,grasp_obj_name))
               rospy.loginfo("Found: %s" % grasp_obj_name)
               obj_tmp = TableObject()
               obj_tmp.graspable_object = grasp_obj
               obj_tmp.graspable_object_name = grasp_obj_name

               if len(grasp_obj.potential_models) > 0:
                   model_index = grasp_obj.potential_models[0].model_id
               else:
                   model_index = -1
               obj_tmp.model_description = self.get_model_description(model_index)

               self.found_objects.append(obj_tmp)

        #if self.raw_objects != None:
        #   self._widget.btn_collision_map.setEnabled(True)
        #self.object_chooser.refresh_list()
        
        #Publish some feedback
        self.server.publish_feedback(ObjectRecognitionFeedback(100, "Model descriptions retrieved from DB. Nb of objects found: " + str(len(self.found_objects))))
        
    def process_collision_map(self):
        res = 0
        try:
            #Args: detection_result reset_collision_models reset_attached_models desired_frame
            res = self.service_tabletop_collision_map.call(self.raw_objects.detection, True, True, "/world")
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: %s" % str(e))

        if res != 0:
            name = res.collision_support_surface_name
            rospy.loginfo("collision_support_surface_name: "+name)
            self.collision_support_surface_name = name
        return res

    def get_model_description(self, model_id):
        """
        Return the object name given it's index (read from database, or
        create unique name if unknown object).
        """
        model = ModelDescription()
        #todo make sure name is unique
        if model_id == -1:
            model.name = "unknown_" + str(self.unknown_object_counter)
            self.unknown_object_counter += 1
        else:
            try:
                db_model_description = self.service_db_get_model_description(model_id)
                model.name = db_model_description.name
                model.tags = db_model_description.tags
                model.maker = db_model_description.maker
            except rospy.ServiceException, e:
                print "Service did not process request: %s" % str(e)
                model.name = "unkown_recognition_failed"
        return model


if __name__ == '__main__':
  rospy.init_node('object_recognition_server')
  server = ObjectRecognitionServer()
  rospy.spin()