#!/usr/bin/env python
#
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
#

import roslib; roslib.load_manifest("sr_pick_and_place")
import rospy

from object_manipulation_msgs.msg import Grasp, PickupGoal, PickupAction, PlaceGoal, PlaceAction
from object_manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest, GraspPlanningResponse
from arm_navigation_msgs.srv import GetMotionPlanRequest, GetMotionPlanResponse, FilterJointTrajectory, FilterJointTrajectoryRequest
from arm_navigation_msgs.msg import DisplayTrajectory, JointLimits
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from sr_utilities.srv import getJointState
from planification import Planification
from sr_utilities.srv import getJointState

import time
import copy

ARM_NAMES = ['ShoulderJRotate', 'ShoulderJSwing', 'ElbowJSwing', 'ElbowJRotate', "WRJ1", "WRJ2"]

class Execution(object):
    """
    """

    def __init__(self, ):
        """
        """
        #initialize the planner
        self.plan = Planification()

        self.display_traj_pub_ = rospy.Publisher("/joint_path_display", DisplayTrajectory, latch=True)
        self.send_traj_pub_ = rospy.Publisher("/command", JointTrajectory, latch=True)
        
        rospy.loginfo("Waiting for services  /getJointState, /trajectory_filter_unnormalizer/filter_trajectory, /database_grasp_planning")
        rospy.wait_for_service("/getJointState")
        rospy.wait_for_service("/trajectory_filter_unnormalizer/filter_trajectory")
        rospy.wait_for_service("/objects_database_node/database_grasp_planning")
        rospy.loginfo("  OK services found")
                
        self.get_joint_state_ = rospy.ServiceProxy("/getJointState", getJointState)
        self.trajectory_filter_ = rospy.ServiceProxy("/trajectory_filter_unnormalizer/filter_trajectory", FilterJointTrajectory)
        self.grasp_planning_service_ = rospy.ServiceProxy("/objects_database_node/database_grasp_planning", GraspPlanning)

                
    def pick(self,pickup_goal):
        #get grasps for the object
          # fill up a grasp planning request
        grasp_planning_req = GraspPlanningRequest()
        grasp_planning_req.arm_name = pickup_goal.arm_name
        grasp_planning_req.target = pickup_goal.target     

         # call grasp planning service
        grasp_planning_res = self.grasp_planning_service_.call(grasp_planning_req) 
        #print grasp_planning_res
         # process grasp planning result
        if (grasp_planning_res.error_code.value != grasp_planning_res.error_code.SUCCESS):
            rospy.logerr("No grasp found for this object, we will generate some, but only when the node is ready for that !")
            return 
        else:
            rospy.loginfo("Got "+ str(len(grasp_planning_res.grasps)) +" grasps for this object")
                
        #for each grasp, check path from pre-grasp pose to grasp pose first and then check motion to pre-grasp pose
        motion_plan_res=GetMotionPlanResponse()
        for index, grasp in enumerate(grasp_planning_res.grasps):
            # extract grasp_pose
            grasp_pose_ = PoseStamped()
            grasp_pose_.header.frame_id = "/world";
            grasp_pose_.pose = grasp.grasp_pose
           
            #pre_grasp_pose_ = PoseStamped()
            #pre_grasp_pose_.header.frame_id = "/world";
            #pre_grasp_pose_.pose.position=grasp_pose_.pose.position
            #pre_grasp_pose_.pose.orientation=grasp_pose_.pose.orientation
            
            pre_grasp_pose_ = copy.deepcopy(grasp_pose_)
            
            # add desired_approach_distance along the approach vector. above the object to plan pre-grasp pose

            # currently add this to Z because approach vector needs to be computer somehow first
            pre_grasp_pose_.pose.position.z = pre_grasp_pose_.pose.position.z + 0.08 
          
            
            # for distance from 0 (grasp_pose) to desired_approach distance (pre_grasp_pose) test IK/Collision and save result
            # decompose this in 10 steps by default
            interpolated_motion_plan_res = self.plan.get_interpolated_ik_motion_plan(pre_grasp_pose_, grasp_pose_, False)
              
            # check the result (depending on number of steps etc...)
            if (interpolated_motion_plan_res.error_code.val == interpolated_motion_plan_res.error_code.SUCCESS):
                number_of_interpolated_steps=0
                for interpolation_index, traj_error_code in enumerate(interpolated_motion_plan_res.trajectory_error_codes):
                    if traj_error_code.val!=1:
                        rospy.logerr("One unfeasible approach-phase step found at "+str(interpolation_index)+ "with val " + str(traj_error_code.val))
                        break
                    else:
                        number_of_interpolated_steps=interpolation_index
                
                if number_of_interpolated_steps+1==len(interpolated_motion_plan_res.trajectory.joint_trajectory.points):
                    rospy.loginfo("Grasp number "+str(index)+" approach is possible, checking motion plan to pre-grasp")
                    #print interpolated_motion_plan_res
                
                    # check and plan motion to this pre_grasp_pose
                    motion_plan_res = self.plan.plan_arm_motion( pickup_goal.arm_name, "jointspace", pre_grasp_pose_ )
        
                    #if one is successful do not test others
                    if (motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS):
                        rospy.loginfo("Grasp number "+str(index)+" is possible, executing it")
                        break
                else:
                    rospy.logerr("Grasp number "+str(index)+" approach is impossible")
                    #print interpolated_motion_plan_res
            else:
                rospy.logerr("Grasp number "+str(index)+" approach is impossible")
                #print interpolated_motion_plan_res
            
        #if one is successful, compute approach (from pre-grasp to grasp)
        if (motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS):
            #put hand in pre-grasp posture

            #go there 
            # filter the trajectory
            filtered_traj = self.filter_traj_(motion_plan_res)

            self.display_traj_( filtered_traj )
            self.send_traj_( filtered_traj )

            #approach
            time.sleep(15)
            self.send_traj_( interpolated_motion_plan_res.trajectory.joint_trajectory )
            #grasp
        else:
            rospy.logerr("None of the grasps tested is possible")
              
        
    #def lift(self, target_height):
        #straight movement above object, just for compatibility
        #check if object is grasped
        #compute straight trajectory in 6D or in 3D if it fails
    
   
        
                
    def display_traj_(self, trajectory):
        print "Display trajectory"

        traj = DisplayTrajectory()
        traj.model_id = "shadow"
        traj.trajectory.joint_trajectory = trajectory
        traj.trajectory.joint_trajectory.header.frame_id = "world"
        traj.trajectory.joint_trajectory.header.stamp = rospy.Time.now()
        self.display_traj_pub_.publish(traj)

        print "   -> trajectory published"
        time.sleep(0.5)
        
  
    def filter_traj_(self, motion_plan_res):
        try:
            req = FilterJointTrajectoryRequest()
            for name in ARM_NAMES:
                limit = JointLimits()
                limit.joint_name = name
                limit.min_position = -1.5
                limit.max_position = 1.5
                limit.has_velocity_limits = True
                limit.max_velocity = 0.1
                limit.has_acceleration_limits = True
                limit.max_acceleration = 0.1
                req.limits.append(limit)

            req.trajectory = motion_plan_res.trajectory.joint_trajectory
            req.allowed_time = rospy.Duration.from_sec( 5.0 )

            res = self.get_joint_state_.call()
            req.start_state.joint_state = res.joint_state

            res = self.trajectory_filter_.call( req )
        except rospy.ServiceException, e:
            rospy.logerr("Failed to filter "+str(e))
            return motion_plan_res.trajectory

        return res.trajectory
        
    def send_traj_(self, trajectory):
        print "Sending trajectory"

        traj = trajectory
        for index, point in enumerate(traj.points):
            #if index == 0 or index == len(traj.points) - 1:
            #    point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            #else:
            #    point.velocities = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            point.time_from_start = rospy.Duration.from_sec(float(index) / 8.0)
        #    traj.points[index] = point
        self.send_traj_pub_.publish( traj )

        print "   -> trajectory sent"
        
if __name__ =="__main__":
    rospy.init_node("execution")
    execute = Execution()

    object_pose = PoseStamped()
    object_pose.pose.position.x = 0.470
    object_pose.pose.position.y = 0.166
    object_pose.pose.position.z = 1.665
    
    object_pose.pose.orientation.x = 0.375
    object_pose.pose.orientation.y = 0.155
    object_pose.pose.orientation.z = 0.844
    object_pose.pose.orientation.w = 0.351
    
    execute.pick( object_pose )
    rospy.spin()


