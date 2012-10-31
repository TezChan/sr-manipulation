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
import math

import scipy
import scipy.linalg
import numpy as np

import roslib; roslib.load_manifest('sr_gui_manipulation')
import rospy
from rospy import loginfo, logerr, logdebug
from std_msgs.msg import Float64
from etherCAT_hand_lib import EtherCAT_Hand_Lib
from shadowhand_ros import ShadowHand_ROS

from rosgui.QtBindingHelper import loadUi
import QtGui
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint, SIGNAL
from QtGui import *

from tabletop_object_detector.srv import TabletopDetection
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from household_objects_database_msgs.srv import GetModelDescription
from household_objects_database_msgs.msg import DatabaseModelPose
import object_manipulator.draw_functions as draw_functions
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from object_manipulation_msgs.msg import Grasp, PickupGoal, PickupAction, PickupResult, PlaceGoal, PlaceAction, ManipulationResult, GraspableObject
from object_manipulator.convert_functions import *
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Pose
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
import control_msgs.msg
import trajectory_msgs.msg
from tf import transformations
import tf

import copy

from execution import Execution

import yaml

class ObjectChooser(QWidget):
    """
    Display the list of found objects in a table. Make PickUp calls to
    manipulator action server on selected objects.
    """
    def __init__(self, parent, gui, title):
        QWidget.__init__(self)
        self.gui   = gui
        self.grasp = None
        self.title = QLabel()
        self.title.setText(title)
        self.draw_functions = draw_functions.DrawFunctions('grasp_markers')
        self.pickup_result  = None
        self.listener       = tf.TransformListener()       
        self.grasp_display_publisher = rospy.Publisher("/grasp_display", Grasp)
        self.pickupservice = Execution()

    def draw(self):
        self.frame = QFrame(self)

        self.tree = QTreeWidget()
        self.connect(self.tree,
                     SIGNAL('itemDoubleClicked (QTreeWidgetItem *, int)'),
                     self.double_click)
        self.tree.setHeaderLabels(["Object Name", "Maker", "tags"])
        self.tree.resizeColumnToContents(0)
        self.tree.resizeColumnToContents(1)
        self.tree.resizeColumnToContents(2)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.tree)

        self.frame.setLayout(self.layout)
        layout = QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()

    def double_click(self, item, value):
        self.object_name = str(item.text(0))
        self.object = self.gui.found_objects[self.object_name]

        graspable_object = self.object.graspable_object

        # draw a bounding box around the selected object
        (box_pose, box_dims) = self.call_find_cluster_bounding_box(graspable_object.cluster)
        if box_pose == None:
            return

        box_mat = pose_to_mat(box_pose.pose)
        rospy.logerr("box_pose %f %f %f  q: %f %f %f %f",box_pose.pose.position.x,box_pose.pose.position.y,box_pose.pose.position.z,
        box_pose.pose.orientation.x,box_pose.pose.orientation.y,box_pose.pose.orientation.z,box_pose.pose.orientation.w)
        box_ranges = [[-box_dims.x / 2, -box_dims.y / 2, -box_dims.z / 2],
                      [box_dims.x / 2, box_dims.y / 2, box_dims.z / 2]]
        self.box_pose=box_pose

        self.draw_functions.draw_rviz_box(box_mat, box_ranges, '/world',
                                          ns='bounding box',
                                          color=[0, 0, 1], opaque=0.25, duration=60)

        # call the pickup service
        res = self.pickup(graspable_object, self.object.graspable_object_name, self.object_name)
                
        if res == 0: #correctly picked up
        #TODO: set up place_location
            executed_grasp = self.pickup_result.grasp
            initial_pose = PoseStamped()
            initial_pose.header.stamp = rospy.get_rostime()
            initial_pose.header.frame_id = "/world"
            initial_pose.pose.position.x = self.box_pose.pose.position.x+0.1
            initial_pose.pose.position.y = self.box_pose.pose.position.y-0.3
            initial_pose.pose.position.z = self.box_pose.pose.position.z-box_dims.z/2 # graspable object is from bottom but bounding box is at center !
            q=transformations.quaternion_about_axis(-0.05, (0,0,1))
            initial_pose.pose.orientation.x = self.box_pose.pose.orientation.x #q[0]
            initial_pose.pose.orientation.y = self.box_pose.pose.orientation.y#q[1]
            initial_pose.pose.orientation.z = self.box_pose.pose.orientation.z#q[2]
            initial_pose.pose.orientation.w = self.box_pose.pose.orientation.w#q[3]

            self.list_of_poses = self.compute_list_of_poses(initial_pose, graspable_object, executed_grasp)
            #print "list of pose",self.list_of_poses

            self.place_object(graspable_object, self.object.graspable_object_name, self.object_name, self.list_of_poses)
            

    def place_click(self):
      
        if (self.pickup_result):
            grasp = self.pickup_result.grasp
            self.pickupservice.pre_grasp_exec(grasp,10.0)
        else:
            self.pickupservice.grasp_release_exec(10.0)
        
        initial_pose = PoseStamped()
        initial_pose.header.stamp = rospy.get_rostime()
        initial_pose.header.frame_id = "/world"#"/fixed"
        
        #fake hand to world pose
        initial_pose.pose.position.x = 0.4#self.box_pose.pose.position.x+0.0
        initial_pose.pose.position.y = 0.091# self.box_pose.pose.position.y-0.2
        initial_pose.pose.position.z = 1.25#self.box_pose.pose.position.z+0.05
        q=transformations.quaternion_about_axis(-0.05, (0,0,1))
        initial_pose.pose.orientation.x = 0.41#self.box_pose.pose.orientation.x#q[0]
        initial_pose.pose.orientation.y = 0.695#self.box_pose.pose.orientation.y#q[1]
        initial_pose.pose.orientation.z = 0.52#self.box_pose.pose.orientation.z#q[2]
        initial_pose.pose.orientation.w = 0.284#self.box_pose.pose.orientation.w#q[3]

        executed_grasp=Grasp()
        executed_grasp.grasp_pose=copy.deepcopy(initial_pose.pose)
        
        #fake obj world origin pose
        initial_pose.pose.position.x = 0.43
        initial_pose.pose.position.y = 0.149
        initial_pose.pose.position.z = 1.088
        initial_pose.pose.orientation.x = 0.0#self.box_pose.pose.orientation.x#q[0]
        initial_pose.pose.orientation.y = 0.0#self.box_pose.pose.orientation.y#q[1]
        initial_pose.pose.orientation.z = 0.567#self.box_pose.pose.orientation.z#q[2]
        initial_pose.pose.orientation.w = 0.824#self.box_pose.pose.orientation.w#q[3]
        
        #fake graspable object 
        graspable_object = GraspableObject()
        graspable_object.reference_frame_id = "/world"
        mypotentialmodels = DatabaseModelPose()
        mypotentialmodels.model_id = 18744
        mypotentialmodels.confidence = 1.0
        mypotentialmodels.pose = copy.deepcopy(initial_pose)
        graspable_object.potential_models.append(mypotentialmodels)
        
        #fake obj world destination pose
        initial_pose.pose.position.x = 0.43
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 1.088
        initial_pose.pose.orientation.x = 0.0#self.box_pose.pose.orientation.x#q[0]
        initial_pose.pose.orientation.y = 0.0#self.box_pose.pose.orientation.y#q[1]
        initial_pose.pose.orientation.z = 0.567#self.box_pose.pose.orientation.z#q[2]
        initial_pose.pose.orientation.w = 0.824#self.box_pose.pose.orientation.w#q[3]
        
        list_of_poses = self.compute_list_of_poses(initial_pose, graspable_object, executed_grasp)
        #print "list of pose",list_of_poses

        #self.place_object(self.object.graspable_object, self.object.graspable_object_name, self.object_name, self.list_of_poses)

    def pickup(self, graspable_object, graspable_object_name, object_name):
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
        pickup_goal.collision_support_surface_name = self.gui.collision_support_surface_name

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

    def place_object(self, graspable_object, graspable_object_name, object_name, list_of_poses ):
        """
        Place the given object in the given pose
        """
        if self.pickup_result == None:
            rospy.logwarn("No objects where picked up. Aborting place object action.")
            return

        info_tmp = "Placing "+object_name
        rospy.loginfo(info_tmp)
        place_goal = PlaceGoal()

        #place at the prepared location
        place_goal.place_locations = list_of_poses

        place_goal.collision_object_name = graspable_object_name
        place_goal.collision_support_surface_name = self.gui.collision_support_surface_name
        print "collision support surface name: ",self.gui.collision_support_surface_name

        #information about which grasp was executed on the object,
        #returned by the pickup action
        place_goal.grasp = self.pickup_result.grasp
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
            loginfo("Place succeeded, now retreating")
            self.pickupservice.retreat(0.07)
        else:
            loginfo("Place failed")
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
        (trans_palm,rot_palm) = self.listener.lookupTransform('/world', '/palm', rospy.Time(0))
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

    def refresh_list(self, value=0):
        self.tree.clear()
        first_item = None
        object_names = self.gui.found_objects.keys()
        object_names.sort()
        for object_name in object_names:
            item = QTreeWidgetItem(self.tree)
            if first_item == None:
                first_item = item

            item.setText(0, object_name)
            obj = self.gui.found_objects[object_name].model_description
            if "unknown_" not in object_name:
                item.setText(1, obj.maker)

                tags = ""
                for tag in obj.tags:
                    tags += str(tag) + " ; "
                item.setText(2, tags)

            self.tree.resizeColumnToContents(0)
            self.tree.resizeColumnToContents(1)
            self.tree.resizeColumnToContents(2)
        return first_item


class TableObject(object):
    """
    Contains all the relevant info for an object.
    A map of TableObject is stored in SrGuiManipulation.
    """
    def __init__(self):
        self.graspable_object      = None
        self.graspable_object_name = None
        self.model_description     = None


class Model(object):
    def __init__(self):
        self.name = None


class SrGuiManipulation(QObject):
    """The main GUI dock window"""
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

        self_dir        = os.path.dirname(os.path.realpath(__file__));
        self.config_dir = os.path.join(self_dir, '../config')
        self.ui_dir     = os.path.join(self_dir, '../ui')

        # UI setup
        ui_file = os.path.join(self.ui_dir, 'SrGuiManipulation.ui')

        main_window = plugin_context.main_window()
        self.win = QDockWidget(main_window)
        loadUi(ui_file, self.win)
        if plugin_context.serial_number() > 1:
            self.win.setWindowTitle(
                    self.win.windowTitle() + (' (%d)' % plugin_context.serial_number()) )
        main_window.addDockWidget(Qt.RightDockWidgetArea, self.win)

        # trigger deleteLater for plugin when win is closed
        self.win.installEventFilter(self)

        self.object_chooser = ObjectChooser(self.win, self, "Objects Detected")
        self.win.contents.layout().addWidget(self.object_chooser)
        self.object_chooser.draw()

        # Bind button clicks
        self.win.btn_detect_objects.pressed.connect(self.detect_objects)
        self.win.btn_collision_map.pressed.connect(self.process_collision_map)
        self.win.btn_collision_map.setEnabled(False)
        self.win.btn_start_grab_position.pressed.connect(self.start_grab_position)
#        self.win.btn_zero_position.pressed.connect(self.zero_position)
        self.win.btn_zero_position.pressed.connect(self.redo_place)
        self.robot_lib_eth = EtherCAT_Hand_Lib()

# Guillaume: Currently removed because requires ethercat to be active to run, ethercat does exist in sim
#        if not self.robot_lib_eth.activate_joint_states():
#            logerr("The EtherCAT Hand node doesn't seem to be running")
#        self.robot_lib_can = ShadowHand_ROS()
#        if not self.robot_lib_can.has_arm():
#            logerr("The CAN Arm node doesn't seem to be running")

        self.init_services()
        self.init_joint_pubs()

    def init_services(self):
        """Service setup"""
        srvname = '/tabletop_collision_map_processing/tabletop_collision_map_processing'
        rospy.wait_for_service(srvname)
        self.service_tabletop_collision_map = rospy.ServiceProxy(srvname, TabletopCollisionMapProcessing)

        srvname = 'object_detection'
        rospy.wait_for_service(srvname)
        self.service_object_detector = rospy.ServiceProxy(srvname, TabletopDetection)

        srvname = 'objects_database_node/get_model_description'
        rospy.wait_for_service(srvname)
        self.service_db_get_model_description = rospy.ServiceProxy(srvname, GetModelDescription)

    def init_joint_pubs(self):
        """Setup publishers for the arm and hand joints"""
        self.joint_pub = {};
        # Hand joints
        for j in [
            "ffj0", "ffj3", "ffj4",
            "lfj0", "lfj3", "lfj4", "lfj5",
            "mfj0", "mfj3", "mfj4",
            "rfj0", "rfj3", "rfj4",
            "thj1", "thj2", "thj3", "thj4", "thj5",
            "wrj1", "wrj2"
            ]:
            topic = 'sh_'+j+'_mixed_position_velocity_controller/command'
            self.joint_pub[j] = rospy.Publisher(topic, Float64)
        # Arm joints
        for j in ["er", "es", "sr", "ss"]:
            topic = 'sa_'+j+'_position_controller/command'
            self.joint_pub[j] = rospy.Publisher(topic, Float64)

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
        self.win.contents.setCursor(Qt.WaitCursor)
        try:
            self.raw_objects = self.service_object_detector(True, True, 1)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)

        #print self.raw_objects
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
        # TODO: Should really do this with SIGNALs and SLOTs.
        self.object_chooser.refresh_list()
        self.win.contents.setCursor(Qt.ArrowCursor)

    def process_collision_map(self):
        res = 0
        try:
            #Args: detection_result reset_collision_models reset_attached_models desired_frame
            res = self.service_tabletop_collision_map.call(self.raw_objects.detection, True, True, "/world")
        except rospy.ServiceException, e:
            logerr("Service did not process request: %s" % str(e))

        if res != 0:
            name = res.collision_support_surface_name
            loginfo("collision_support_surface_name: "+name)
            self.collision_support_surface_name = name
            self.win.collision_support_surface_name.setText(name)
        return res

    def get_object_name(self, model_id):
        """
        Return the object name given it's index (read from database, or
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

    def position_arm(self, pos_name):
        """Move the arm through a named series of positions defined in YAML config.
        Loads config on each call so that GUI will pickup changes to the config file.
        """
        yaml_file = os.path.join(self.config_dir, 'positions.yaml')
        try:
            data = yaml.load(open(yaml_file))
        except IOError as err:
            QMessageBox.warning(self.win, "Warning",
                    "Failed to load '"+yaml_file+"': "+str(err))
            return

        self.win.contents.setCursor(Qt.WaitCursor)
        try:
            self._position_arm_actionlib(data[pos_name])
            #self._position_arm_direct(data[pos_name])
        except:
            self.win.contents.setCursor(Qt.ArrowCursor)
            raise
        self.win.contents.setCursor(Qt.ArrowCursor)

    def _position_arm_actionlib(self, moves):
        # Use action server r_arm_controller/joint_trajectory_action
        # FollowJointTrajectoryGoal needs a fixed list of joint names,
        # specified for all steps
        traj = { 'joint_names': [], 'points': [] }
        # Scan list of moves to get all joint names.
        jnames_map = {}
        for step in moves:
            for k in step['positions'].keys():
                jnames_map[k] = 1
        jnames = jnames_map.keys()
        traj['joint_names'] = jnames
        # Any joints not specified in fist step need to get positions read
        # Remember last position for each joint while generating points so we
        # can fill them in  when not given in YAML step.
        joint_last_pos = {}
        for jn in jnames:
            if jn in moves[0]['positions']:
                joint_last_pos[jn] = math.radians(moves[0]['positions'][jn])
            else:
                joint_last_pos[jn] = self.current_joint_pos(jn)
        # Fill out the full trajectory
        time_from_start = 0
        for step in moves:
            positions = []
            time_from_start += step['wait']
            point = {
                'time_from_start': rospy.Duration(time_from_start),
                'positions':       positions }
            traj['points'].append(point)
            for jn in jnames:
                if jn in step['positions']:
                    pos = math.radians(step['positions'][jn])
                    positions.append(pos)
                    joint_last_pos[jn] = pos
                else:
                    positions.append( joint_last_pos[jn] )
        print "Converted trajectory:", traj

        # Create and send goal
        client = actionlib.SimpleActionClient(
                "r_arm_controller/joint_trajectory_action",
                control_msgs.msg.FollowJointTrajectoryAction)
        client.wait_for_server()
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = traj['joint_names']
        for pt in traj['points']:
            pt_msg = trajectory_msgs.msg.JointTrajectoryPoint()
            pt_msg.positions = pt['positions']
            pt_msg.time_from_start = pt['time_from_start']
            goal.trajectory.points.append(pt_msg)
        client.send_goal(goal)
        # No need to actually wait, can let action server run, allows us to
        # preempt with another motion or cancel
        #client.wait_for_result()

    def current_joint_pos(self, jname):
        """Return current position of a joint name (hand or arm) in radians"""
        arm_name_map = {
            'sr': "ShoulderJRotate",
            'ss': "ShoulderJSwing",
            'es': "ElbowJSwing",
            'er': "ElbowJRotate" }
        if jname in ["er", "es", "sr", "ss"]: # Arm joints
            return math.radians( self.robot_lib_can.valueof(arm_name_map[jname]) )
        else:
            # TODO - Double check this returns in radians.
            return self.robot_lib_eth.get_position(jname.upper())

    def _position_arm_direct(self, traj):
        for step in traj:
            rospy.sleep(step['wait'])
            print(str(step['wait'])+": "+str(step['positions']))
            for jname, pos_degs in step['positions'].iteritems():
                pos_rads = math.radians(pos_degs);
                self.joint_pub[jname].publish(Float64(pos_rads))

    def zero_position(self):
        """Move all joints back to zero"""
        self.position_arm('zero')

    def redo_place(self):
        self.object_chooser.place_click()

    def start_grab_position(self):
        """Move the arm into a good starting position for the grab"""
        #self.position_arm('start_grab')
        


