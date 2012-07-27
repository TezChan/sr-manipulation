#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_grasp_viz')
import rospy
from rospy import loginfo, logerr, logdebug
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Pose
from object_manipulation_msgs.msg import Grasp, GraspResult
import tf

RATE=10

class DisplayGrasp:
  def __init__(self):
    rospy.init_node('display_grasp_service', anonymous=True)
    self.subs = rospy.Subscriber("/grasp_display", Grasp, self.DisplayGraspCB)
    self.grasp_posture_publisher = rospy.Publisher("/grasp/joint_states", JointState)
    self.broadcaster = tf.TransformBroadcaster()
    self.grasp = []
    r = rospy.Rate( RATE )
   
    while not rospy.is_shutdown():
      if self.grasp!=[]:
        self.grasp_posture_publisher.publish(self.grasp.grasp_posture)
        myposition=self.grasp.grasp_pose.position
        myorientation=self.grasp.grasp_pose.orientation
        self.broadcaster.sendTransform((myposition.x, myposition.y, myposition.z),(myorientation.x,myorientation.y,myorientation.z,myorientation.w),rospy.Time.now(),"/grasp/palm","/world")
      r.sleep()
            
  def DisplayGraspCB(self,msg):
    self.grasp=msg


if __name__ == '__main__':
    service = DisplayGrasp()



