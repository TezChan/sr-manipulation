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

from handle_demo_step_actions.msg import PlacePositionSelectionAction, PlacePositionSelectionResult, PlacePositionSelectionFeedback
from handle_demo_step_actions.srv import PlacePositionSelection

class PlacePositionSelectionServer(object):
    def __init__(self):
        self.service_place_position_selection_from_list = None
        self.server = actionlib.SimpleActionServer('select_place_position', PlacePositionSelectionAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Step actionlib servers: place_position_selection server ready")
 
    def init_services(self):
        """Service setup"""
        srvname = '/decision_services/place_position_selection'
        rospy.loginfo("Waiting for service " + srvname)
        rospy.wait_for_service(srvname)
        self.service_place_position_selection_from_list = rospy.ServiceProxy(srvname, PlacePositionSelection)
           
    def execute(self, goal):
        self.server.publish_feedback(PlacePositionSelectionFeedback(1, "Select a pose"))
        
        self.init_services()
        
        try:
            self.selection_srv_response = self.service_place_position_selection_from_list(goal.initial_pose_of_the_object)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
            self.server.publish_feedback(PlacePositionSelectionFeedback(100, "Error: position selection service error"))
            self.server.set_aborted()
            return
        
         
        self.server.publish_feedback(PlacePositionSelectionFeedback(100, "Position selected: x=" + str(self.selection_srv_response.selected_place_pose.pose.position.x) + 
                                                                    " y=" + str(self.selection_srv_response.selected_place_pose.pose.position.y) +
                                                                    " z=" + str(self.selection_srv_response.selected_place_pose.pose.position.z)))

        self.server.set_succeeded(PlacePositionSelectionResult(self.selection_srv_response.selected_place_pose))


if __name__ == '__main__':
    rospy.init_node('place_position_selection_server')
    server = PlacePositionSelectionServer()
    rospy.spin()