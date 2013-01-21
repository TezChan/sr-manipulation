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

import object_manipulator.draw_functions as draw_functions

from rospy import loginfo, logerr, logdebug
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from object_manipulation_msgs.msg import Grasp, PickupGoal, PickupAction, PickupResult, PlaceGoal, PlaceAction, ManipulationResult, GraspableObject
from object_manipulator.convert_functions import *
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Pose
from handle_demo_step_actions.msg import PickObjectAction, PickObjectResult, PickObjectFeedback
from sr_pick_and_place.execution import Execution


class PickServer(object):
    def __init__(self):
        self.draw_functions = draw_functions.DrawFunctions('grasp_markers')
        
        self.pickupservice = Execution()
        self.object = None
        self.object_name = None
        self.server = actionlib.SimpleActionServer('pick_object', PickObjectAction, self.execute, False)
        self.server.start()
        loginfo("Step actionlib servers: pick server ready")
    
    def execute(self, goal):
        (res, initial_pose, executed_grasp) = self.pick_object(goal.object_to_pick, goal.collision_support_surface_name)
        if self.server.is_active():
            if res == 0:
                self.server.publish_feedback(PickObjectFeedback(100, "Pickup succeeded"))
                self.server.set_succeeded(PickObjectResult(res, initial_pose, executed_grasp))
            else:
                rospy.logwarn("Pickup failed")
                self.server.publish_feedback(PickObjectFeedback(100, "Error: Pickup failed"))
                self.server.set_succeeded(PickObjectResult(res, initial_pose, executed_grasp))
        
    def pick_object(self, object_to_pick, collision_support_surface_name):
        self.server.publish_feedback(PickObjectFeedback(1, "Finding object bounding box"))
        self.object_name = object_to_pick.model_description.name
        self.object = object_to_pick

        graspable_object = self.object.graspable_object

        # draw a bounding box around the selected object
        (box_pose, box_dims) = self.call_find_cluster_bounding_box(graspable_object.cluster)
        if box_pose == None:
            return (1, None, None)

        box_mat = pose_to_mat(box_pose.pose)
        rospy.logerr("box_pose %f %f %f  q: %f %f %f %f",box_pose.pose.position.x,box_pose.pose.position.y,box_pose.pose.position.z,
        box_pose.pose.orientation.x,box_pose.pose.orientation.y,box_pose.pose.orientation.z,box_pose.pose.orientation.w)
        box_ranges = [[-box_dims.x / 2, -box_dims.y / 2, -box_dims.z / 2],
                      [box_dims.x / 2, box_dims.y / 2, box_dims.z / 2]]
        self.box_pose=box_pose

        self.draw_functions.draw_rviz_box(box_mat, box_ranges, '/world',
                                          ns='bounding box',
                                          color=[0, 0, 1], opaque=0.25, duration=60)

        if self.server.is_preempt_requested():
            self.server.publish_feedback(PickObjectFeedback(50, "Pick action cancelled"))
            self.server.set_preempted()
            return (1, None, None)
        self.server.publish_feedback(PickObjectFeedback(10, "Calling pick up service"))
        # call the pickup service
        res = self.pickup(graspable_object, self.object.graspable_object_name, self.object_name, collision_support_surface_name)
        
        initial_pose = PoseStamped()
        initial_pose.header.stamp = rospy.get_rostime()
        initial_pose.header.frame_id = "/world"
        initial_pose.pose.position.x = self.box_pose.pose.position.x
        initial_pose.pose.position.y = self.box_pose.pose.position.y
        initial_pose.pose.position.z = self.box_pose.pose.position.z-box_dims.z/2 # graspable object is from bottom but bounding box is at center !

        initial_pose.pose.orientation.x = self.box_pose.pose.orientation.x 
        initial_pose.pose.orientation.y = self.box_pose.pose.orientation.y
        initial_pose.pose.orientation.z = self.box_pose.pose.orientation.z
        initial_pose.pose.orientation.w = self.box_pose.pose.orientation.w
        
        if res == 0: #correctly picked up
            executed_grasp = self.pickup_result.grasp
        else:
            executed_grasp = Grasp()
            
        return (res, initial_pose, executed_grasp)
    
    def pickup(self, graspable_object, graspable_object_name, object_name, collision_support_surface_name):
        """
        Try to pick up the given object. Sends a message (PickupGoal from
        actionlib_msgs) to the manipularior action server on
        /object_manipulator/object_manipulator_pickup/goal
        """
        info_tmp = "Picking up "+ object_name
        rospy.loginfo(info_tmp)
        pickup_goal = PickupGoal()
        pickup_goal.target = graspable_object
        pickup_goal.collision_object_name = graspable_object_name
        pickup_goal.collision_support_surface_name = collision_support_surface_name

        #pickup_goal.additional_link_padding
        pickup_goal.ignore_collisions = True

        pickup_goal.arm_name = "right_arm"
        #pickup_goal.desired_approach_distance = 0.08 This does not exist anymore in the message
        #pickup_goal.min_approach_distance = 0.02 This does not exist anymore in the message

        direction = Vector3Stamped()
        direction.header.stamp = rospy.get_rostime()
        direction.header.frame_id = "/base_link";
        direction.vector.x = 0;
        direction.vector.y = 0;
        direction.vector.z = 1;
        pickup_goal.lift.direction = direction;
        #request a vertical lift of 15cm after grasping the object
        pickup_goal.lift.desired_distance = 0.1;
        pickup_goal.lift.min_distance = 0.07;
        #do not use tactile-based grasping or tactile-based lift
        pickup_goal.use_reactive_lift = True;
        pickup_goal.use_reactive_execution = True;

        self.pickup_result = self.pickupservice.pick(pickup_goal)

        #pickup_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_pickup', PickupAction)
        #pickup_client.wait_for_server()
        #rospy.loginfo("Pickup server ready")

        #pickup_client.send_goal(pickup_goal)
        #TODO: change this when using the robot
        #pickup_client.wait_for_result(timeout=rospy.Duration.from_sec(3600.0))
        loginfo("Got Pickup results")
        #self.pickup_result = pickup_client.get_result()

        #print "Pickup result: "+str(self.pickup_result)
        '''
        if pickup_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("The pickup action has failed: " + str(self.pickup_result.manipulation_result.value) )
            QMessageBox.warning(self, "Warning",
                    "Pickup action failed: "+str(self.pickup_result.manipulation_result.value))
            for tested_grasp,tested_grasp_result in zip(self.pickup_result.attempted_grasps,self.pickup_result.attempted_grasp_results):
                if tested_grasp_result.result_code==7:
                  self.grasp_display_publisher.publish(tested_grasp)
            return -1
        '''

        if self.pickup_result.manipulation_result.value == ManipulationResult.SUCCESS:
            loginfo("Pick succeeded, now lifting")
            self.pickupservice.lift(0.07)

        else:
            loginfo("Pick failed")
            return 1
        return 0

    def call_find_cluster_bounding_box(self, cluster):
        req = FindClusterBoundingBoxRequest()
        req.cluster = cluster
        service_name = "find_cluster_bounding_box"
        rospy.loginfo("waiting for find_cluster_bounding_box service")
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found")
        serv = rospy.ServiceProxy(service_name, FindClusterBoundingBox)
        try:
            res = serv(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling find_cluster_bounding_box: %s" % e)
            return 0
        if not res.error_code:
            return (res.pose, res.box_dims)
        else:
            return (None, None)

if __name__ == '__main__':
    rospy.init_node('pick_server')
    server = PickServer()
    rospy.spin()