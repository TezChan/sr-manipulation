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

from handle_demo_step_actions.msg import ObjectSelectionAction, ObjectSelectionResult, ObjectSelectionFeedback
from handle_demo_step_actions.srv import ObjectSelectionFromList

class ObjectSelectionServer(object):
    def __init__(self):
        self.service_object_selection_from_list = None
        self.selected_index = None
        self.server = actionlib.SimpleActionServer('select_object', ObjectSelectionAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Step actionlib servers: object_selection server ready")
        
    def init_services(self):
        """Service setup"""
        srvname = '/decision_services/object_selection'
        rospy.loginfo("Waiting for service " + srvname)
        rospy.wait_for_service(srvname)
        self.service_object_selection_from_list = rospy.ServiceProxy(srvname, ObjectSelectionFromList)
    
    def execute(self, goal):
        self.server.publish_feedback(ObjectSelectionFeedback(1, "Select an object from the list"))
        if len(goal.objects) == 0:
            rospy.logwarn("Empty object selection list")
            self.server.publish_feedback(ObjectSelectionFeedback(100, "Error: Empty object selection list"))
            self.server.set_aborted()
            return
        
        self.init_services()
        
        try:
            self.object_selection_response = self.service_object_selection_from_list(goal.objects)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
            self.server.publish_feedback(ObjectSelectionFeedback(100, "Error: object selection service error"))
            self.server.set_aborted()
            return
        
        if self.object_selection_response.selected_object_index >= len(goal.objects):
            rospy.logwarn("Invalid index selected: " + str(self.object_selection_response.selected_object_index))
            self.server.publish_feedback(ObjectSelectionFeedback(100, "Error: Invalid index selected: " + str(self.object_selection_response.selected_object_index)))
            self.server.set_aborted()
            return
         
        self.server.publish_feedback(ObjectSelectionFeedback(100, "Object selected: " + goal.objects[self.object_selection_response.selected_object_index].model_description.name))

        self.server.set_succeeded(ObjectSelectionResult(goal.objects[self.object_selection_response.selected_object_index]))


if __name__ == '__main__':
    rospy.init_node('object_selection_server')
    server = ObjectSelectionServer()
    rospy.spin()