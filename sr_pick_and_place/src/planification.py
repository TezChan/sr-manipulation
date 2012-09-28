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

from arm_navigation_msgs.srv import GetMotionPlan, SetPlanningSceneDiff
from arm_navigation_msgs.msg import MotionPlanRequest, Shape, PositionConstraint, OrientationConstraint, DisplayTrajectory, Constraints, JointConstraint
from kinematics_msgs.srv import GetConstraintAwarePositionIK, GetPositionIK
from kinematics_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

import time

class Planification(object):
    """
    """

    def __init__(self, ):
        """
        """
        self.display_traj_pub_ = rospy.Publisher("/joint_path_display", DisplayTrajectory, latch=True)
        self.send_traj_pub_ = rospy.Publisher("/command", JointTrajectory, latch=True)

        rospy.loginfo("Waiting for services /ompl_planning/plan_kinematic_path, /environment_server/set_planning_scene_diff, /shadow_right_arm_kinematics/get_constraint_aware_ik ...")
        rospy.wait_for_service("/ompl_planning/plan_kinematic_path")
        rospy.wait_for_service("/environment_server/set_planning_scene_diff")
        rospy.wait_for_service("/shadow_right_arm_kinematics/get_constraint_aware_ik")
        rospy.loginfo("  OK services found")

        self.set_planning_scene_diff_ = rospy.ServiceProxy("/environment_server/set_planning_scene_diff", SetPlanningSceneDiff)
        self.planifier_ = rospy.ServiceProxy('/ompl_planning/plan_kinematic_path', GetMotionPlan)
        self.constraint_aware_ik_ = rospy.ServiceProxy("/shadow_right_arm_kinematics/get_constraint_aware_ik", GetConstraintAwarePositionIK)
        #self.constraint_aware_ik_ = rospy.ServiceProxy("/shadow_right_arm_kinematics/get_ik", GetPositionIK)

    def plan_pickup(self, object_pose):
        """
        """

        goal = PoseStamped()
        goal.header.frame_id = "world"
        #goal.pose.position.x = 0.399
        #goal.pose.position.y = 0.110
        #goal.pose.position.z = 1.205

        goal.pose.position.x = 0.470
        goal.pose.position.y = -0.156
        goal.pose.position.z = 1.465

        goal.pose.orientation.x = 0.375
        goal.pose.orientation.y = 0.155
        goal.pose.orientation.z = 0.844
        goal.pose.orientation.w = 0.351

        self.plan_motion_joint_state_( goal )

    def plan_motion_joint_state_(self, goal_pose, link_name = "palm"):
        self.reset_planning_scene_()

        #first get the ik for the pose we want to go to
        ik_solution = None
        try:
            req = PositionIKRequest()
            req.ik_link_name = link_name
            req.pose_stamped = goal_pose
            req.ik_seed_state.joint_state.name = ['ShoulderJRotate', 'ShoulderJSwing', 'ElbowJSwing', 'ElbowJRotate', "WRJ1", "WRJ2"]
            req.ik_seed_state.joint_state.position = [-0.0011556513918362654, 0.33071140061761284, 1.9152039367468952, 0.008440334101951663, 0.0, 0.0]
            ik_solution = self.constraint_aware_ik_.call( req, Constraints(), rospy.Duration(5.0) )
        except rospy.ServiceException, e:
            rospy.logerr( "Failed to compute IK "+str(e) )
            return False

        if ik_solution.error_code.val != ik_solution.error_code.SUCCESS:
            rospy.logerr("couldn't find an ik solution to go to: " + str(goal_pose))
            return False

        motion_plan_res = None
        try:
            motion_plan_request = MotionPlanRequest()

            motion_plan_request.group_name = "right_arm"
            motion_plan_request.num_planning_attempts = 1
            motion_plan_request.planner_id = ""
            motion_plan_request.allowed_planning_time = rospy.Duration(5.0)

            for target, name in zip(ik_solution.solution.joint_state.position, ik_solution.solution.joint_state.name):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = name
                joint_constraint.position = target
                joint_constraint.tolerance_below = 0.1
                joint_constraint.tolerance_above = 0.1

                motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint)

            motion_plan_res = self.planifier_( motion_plan_request )

            if motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS:
                self.display_traj_( motion_plan_res )
                self.send_traj_( motion_plan_res )
            else:
                rospy.logerr("The planning failed: " + str(motion_plan_res.error_code.val))

        except rospy.ServiceException, e:
            rospy.logerr( "Failed to plan "+str(e) )
            return False




    def plan_motion_cartesian_(self, goal_pose, link_name="palm"):
        self.reset_planning_scene_()

        motion_plan_res = None
        try:
            motion_plan_request = MotionPlanRequest()

            motion_plan_request.group_name = "right_arm_cartesian"
            motion_plan_request.num_planning_attempts = 1
            motion_plan_request.planner_id = ""
            motion_plan_request.allowed_planning_time = rospy.Duration(120.0)

            position_constraint = PositionConstraint()
            position_constraint.header.stamp = rospy.Time.now()
            position_constraint.header.frame_id = goal_pose.header.frame_id
            position_constraint.link_name = link_name
            position_constraint.position = goal_pose.pose.position
            position_constraint.constraint_region_shape.type = Shape.BOX
            position_constraint.constraint_region_shape.dimensions.append(0.1)
            position_constraint.constraint_region_shape.dimensions.append(0.1)
            position_constraint.constraint_region_shape.dimensions.append(0.1)
            position_constraint.weight = 1.0
            #position_constraint.constraint_region_orientation.x = 0.307
            #position_constraint.constraint_region_orientation.y = 0.127
            #position_constraint.constraint_region_orientation.z = 0.871
            #position_constraint.constraint_region_orientation.w = 0.362
            position_constraint.constraint_region_orientation.w = 1.0
            motion_plan_request.goal_constraints.position_constraints.append(position_constraint)

            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.stamp = rospy.Time.now()
            orientation_constraint.header.frame_id = goal_pose.header.frame_id
            orientation_constraint.link_name = link_name

            orientation_constraint.orientation.x = 0.351
            orientation_constraint.orientation.y = 0.155
            orientation_constraint.orientation.z = 0.844
            orientation_constraint.orientation.w = 0.375

            #orientation_constraint.orientation.x = 0.454
            #orientation_constraint.orientation.y = 0.665
            #orientation_constraint.orientation.z = 0.499
            #orientation_constraint.orientation.w = 0.320

            orientation_constraint.absolute_roll_tolerance = 1.5
            orientation_constraint.absolute_pitch_tolerance = 1.5
            orientation_constraint.absolute_yaw_tolerance = 1.5
            orientation_constraint.weight = 0.5
            motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

            motion_plan_res = self.planifier_( motion_plan_request )

            if motion_plan_res.error_code.val == motion_plan_res.error_code.SUCCESS:
                self.display_traj_( motion_plan_res )
            else:
                rospy.logerr("The planning failed: " + str(motion_plan_res.error_code.val))

        except rospy.ServiceException, e:
            rospy.logerr( "Failed to plan "+str(e) )
            return False

    def display_traj_(self, motion_plan_result):
        print "Display trajectory"

        traj = DisplayTrajectory()
        traj.model_id = "shadow"
        traj.trajectory.joint_trajectory.header.frame_id = "world"
        traj.trajectory.joint_trajectory.header.stamp = rospy.Time.now()
        traj.trajectory = motion_plan_result.trajectory

        self.display_traj_pub_.publish(traj)

        print "   -> trajectory published"
        time.sleep(0.5)


    def send_traj_(self, motion_plan_result):
        print "Sending trajectory"

        traj = motion_plan_result.trajectory.joint_trajectory
        for index, point in enumerate(traj.points):
            if index == 0 or index == len(traj.points) - 1:
                point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            else:
                point.velocities = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
            point.time_from_start = rospy.Duration.from_sec(float(index) / 8.0)
            traj.points[index] = point
        self.send_traj_pub_.publish( traj )

        print "   -> trajectory sent"

    def reset_planning_scene_(self):
        try:
            self.set_planning_scene_diff_.call()
        except:
            rospy.logerr("failed to set the planning scene")

if __name__ =="__main__":
    rospy.init_node("planification")
    plan = Planification()

    object_pose = PoseStamped()
    object_pose.pose.position.x = 0.45
    object_pose.pose.position.y = 0.16
    object_pose.pose.position.z = 1.2

    plan.plan_pickup( object_pose )
    rospy.spin()

