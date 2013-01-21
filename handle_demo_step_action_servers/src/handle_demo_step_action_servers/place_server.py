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
import tf
import object_manipulator.draw_functions as draw_functions
import copy

from object_manipulation_msgs.msg import Grasp, PickupGoal, PickupAction, PickupResult, PlaceGoal, PlaceAction, ManipulationResult, GraspableObject
from object_manipulator.convert_functions import *
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Pose
from handle_demo_step_actions.msg import PlaceObjectAction, PlaceObjectResult, PlaceObjectFeedback
from sr_pick_and_place.execution import Execution

class PlaceServer(object):
    def __init__(self):
        self.draw_functions = draw_functions.DrawFunctions('grasp_markers')
        #We're creating another Execution object here (there's another one in pick_server.py)
        #We need to make sure that they don't interfere with each other
        #(Execution doesn't provide any service, so it shouldn't be a problem)
        self.pickupservice = Execution()
        self.listener = self.pickupservice.listener
        
        self.server = actionlib.SimpleActionServer('place_object', PlaceObjectAction, self.execute, False)
        self.server.start()
        
        rospy.loginfo("Step actionlib servers: place server ready")
    
    def execute(self, goal):
        res = self.place(goal.place_pose, goal.object_to_place, goal.executed_grasp, goal.collision_support_surface_name)
        if self.server.is_active():
            if res == 0:
                self.server.publish_feedback(PlaceObjectFeedback(100, "Place object succeeded"))
                self.server.set_succeeded(PlaceObjectResult(res))
            else:
                rospy.logwarn("Place failed")
                self.server.publish_feedback(PlaceObjectFeedback(100, "Error: Place object failed"))
                self.server.set_succeeded(PlaceObjectResult(res))
        
    def place(self, place_pose, object_to_place, executed_grasp, collision_support_surface_name):
        self.server.publish_feedback(PlaceObjectFeedback(1, "Computing list of poses"))
        list_of_poses = self.compute_list_of_poses(place_pose, object_to_place.graspable_object, executed_grasp)
        
        if self.server.is_preempt_requested():
            self.server.publish_feedback(PlaceObjectFeedback(50, "Place action cancelled"))
            self.server.set_preempted()
            return 1
        
        self.server.publish_feedback(PlaceObjectFeedback(25, "Placing object"))
        res = self.place_object(object_to_place.graspable_object, object_to_place.graspable_object_name, object_to_place.model_description.name, list_of_poses, executed_grasp, collision_support_surface_name)
        return res

    def place_object(self, graspable_object, graspable_object_name, object_name, list_of_poses, executed_grasp, collision_support_surface_name ):
        """
        Place the given object in the given pose
        """
        '''
        if self.pickup_result == None:
            rospy.logwarn("No objects where picked up. Aborting place object action.")
            return
        '''
        info_tmp = "Placing "+object_name
        rospy.loginfo(info_tmp)
        place_goal = PlaceGoal()

        #place at the prepared location
        place_goal.place_locations = list_of_poses

        place_goal.collision_object_name = graspable_object_name
        place_goal.collision_support_surface_name = collision_support_surface_name
        print "collision support surface name: ",collision_support_surface_name

        #information about which grasp was executed on the object,
        #returned by the pickup action
        place_goal.grasp = executed_grasp
        #use the right rm to place
        place_goal.arm_name = "right_arm"
        #padding used when determining if the requested place location
        #would bring the object in collision with the environment
        place_goal.place_padding = 0.01
        #how much the gripper should retreat after placing the object
        place_goal.desired_retreat_distance = 0.1
        place_goal.min_retreat_distance = 0.05
        #we will be putting down the object along the "vertical" direction
        #which is along the z axis in the fixed frame
        direction = Vector3Stamped()
        direction.header.stamp = rospy.get_rostime()
        direction.header.frame_id = "/world"
        direction.vector.x = 0
        direction.vector.y = 0
        direction.vector.z = 1
        place_goal.approach.direction = direction
        place_goal.approach.min_distance = 0.01
        place_goal.approach.desired_distance = 0.03
        #request a vertical put down motion of 10cm before placing the object
        place_goal.desired_retreat_distance = 0.1
        place_goal.min_retreat_distance = 0.05
        #we are not using tactile based placing
        place_goal.use_reactive_place = False


        placeresult = self.pickupservice.place(place_goal)
        '''place_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_place', PlaceAction)
        place_client.wait_for_server()
        rospy.loginfo("Place server ready")

        place_client.send_goal(place_goal)
        #timeout after 1sec
        #TODO: change this when using the robot
        place_client.wait_for_result(timeout=rospy.Duration.from_sec(3600.0))
        rospy.loginfo("Got Place results")

        place_result = place_client.get_result()

        if place_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("The place action has failed: " + str(place_result.manipulation_result.value) )
        print place_result
        '''

        if placeresult.manipulation_result.value == ManipulationResult.SUCCESS:
            rospy.loginfo("Place succeeded, now retreating")
            self.pickupservice.retreat(0.07)
        else:
            rospy.loginfo("Place failed")
            return 1
        return 0


    def compute_list_of_poses(self, destination_pose, graspable_object, executed_grasp):

        list_of_poses = []

        # find hand to object transform
        #  executed_grasp is hand pose in world frame
        #  graspable_object is object in world frame
        hand_world_pose = executed_grasp.grasp_pose
        #print "hand_world", hand_world_pose
        object_world_pose = graspable_object.potential_models[0].pose.pose
        # print "object world" , object_world_pose
        tmathandworld = pose_to_mat(hand_world_pose)
        tmatobjworld = pose_to_mat(object_world_pose)
        hand_object_pose = mat_to_pose(np.dot(np.linalg.inv(tmatobjworld),tmathandworld))
        #print "hand_object_pose" , hand_object_pose

        # generate rotational_symmetric hand to object frames
        hand_object_poses = self.generate_rotational_symmetric_poses(hand_object_pose,16)

        target_pose = PoseStamped()
        target_pose=copy.deepcopy(destination_pose)
        target_pose.header.stamp = rospy.get_rostime()
        #print "target_object_world pose" , target_pose
        tmattarget_world = pose_to_mat(target_pose.pose)

        for hand_object_pose in hand_object_poses:
            tmathandobj = pose_to_mat(hand_object_pose)
            target_hand_posestamped_world = PoseStamped()
            target_hand_posestamped_world.header.frame_id = "/world"
            target_hand_posestamped_world.pose= mat_to_pose(np.dot(tmattarget_world,tmathandobj))
            target_hand_posestamped_world.header.stamp = rospy.get_rostime()
            list_of_poses.append(target_hand_posestamped_world)

        self.draw_place_area(list_of_poses, graspable_object)

        return list_of_poses
    
    def generate_rotational_symmetric_poses(self, hand_object_pose,step=8):
        '''
        Generate a list of 20 pregrasps around the z axis of the object
        '''
        generated_poses = []
        pose=hand_object_pose
        #We'll generate 20 poses centered in 0,0,0 with orientations in 360 degrees around z

        for i in range(0,step):
            angle = (6.28318532 * i)/step
            q = tf.transformations.quaternion_from_euler(0.0,0.0,angle)
            rotation_pose = Pose(Point(0,0,0), Quaternion(0.0,0.0,0.0,1.0))
            rotation_pose.orientation = Quaternion(*q)

            tmatrotz = pose_to_mat(rotation_pose)
            # premultiply by the rotation matrix
            tmatpose = pose_to_mat(pose)
            rotated_pose = mat_to_pose(np.dot(tmatrotz,tmatpose))

            generated_poses.append(rotated_pose)

        return generated_poses
    
    def draw_place_area(self, list_of_poses, graspable_object):
        '''
        Displays all the possible placing locations that are going to be tried.
        '''
        #try:
        (trans_palm,rot_palm) = self.listener.lookupTransform('/world', '/palm', rospy.Time())
        #except:
        #    return

        for index,pose_stamped in enumerate(list_of_poses):
            pose_tmp = Pose()
            pose_tmp.position.x = pose_stamped.pose.position.x
            pose_tmp.position.y = pose_stamped.pose.position.y
            pose_tmp.position.z = pose_stamped.pose.position.z

            pose_tmp.orientation.x = pose_stamped.pose.orientation.x
            pose_tmp.orientation.y = pose_stamped.pose.orientation.y
            pose_tmp.orientation.z = pose_stamped.pose.orientation.z
            pose_tmp.orientation.w = pose_stamped.pose.orientation.w

            mat = pose_to_mat(pose_tmp)
            self.draw_functions.draw_rviz_box(mat, [.01,.01,.01], frame='/world', ns='place_'+str(index),
                                              id=1000+index, duration = 90, color=[0.5,0.5,0.0], opaque=1.0 )


if __name__ == '__main__':
    rospy.init_node('place_server')
    server = PlaceServer()
    rospy.spin()