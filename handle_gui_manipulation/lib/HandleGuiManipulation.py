from __future__ import division
import os
import math

import roslib; roslib.load_manifest('handle_gui_manipulation')
import rospy
from rospy import loginfo, logerr, logdebug
from std_msgs.msg import Float64

from rosgui.QtBindingHelper import loadUi
import QtGui
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint, SIGNAL
from QtGui import *

from tabletop_object_detector.msg import TabletopDetectionResult, Table
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from household_objects_database_msgs.srv import GetModelDescription,GetModelList, GetModelListRequest
import object_manipulator.draw_functions as draw_functions
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest, FindClusterBoundingBoxResponse
from object_manipulation_msgs.msg import PickupGoal, PickupAction, PlaceGoal, PlaceAction, GraspPlanningResult, Grasp
from object_manipulator.convert_functions import *
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Pose, Vector3, Point, Twist, Quaternion
import actionlib
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from synergy_grasp_msgs.msg import SynergyGraspAction, SynergyGraspGoal
import control_msgs.msg
import trajectory_msgs.msg
from sensor_msgs.msg import PointCloud, JointState
from tf import transformations
import tf

from std_srvs.srv import Empty,EmptyRequest

from kcl_msgs.srv import KCL_Sensor_Bias,KCL_Sensor_BiasRequest, KCL_PoseRectifyEReq, KCL_PoseRectifyEReqRequest
from uc3m_msgs.srv import GetObjectPose, GetObjectPoseRequest, GetObjectPoseResponse
from uc3m_msgs.srv import GetCentralObjectOnTable, GetCentralObjectOnTableRequest, getModelbyAcquisition, getModelbyAcquisitionRequest, getModelbyAcquisitionResponse
from geometric_planner_msgs.srv import execute_in_hand_mvt,  execute_in_hand_mvtRequest, execute_in_hand_mvtResponse
from sr_utilities.srv import SetTrackedObject, SetTrackedObjectRequest, GraspRotation, GraspRotationRequest

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
        object_name = str(item.text(0))
        self.object = self.gui.found_objects[object_name]

        graspable_object = self.object.graspable_object

        # draw a bounding box around the selected object
        (box_pose, box_dims) = self.call_find_cluster_bounding_box(graspable_object.cluster)
        if box_pose == None:
            return

        box_mat = pose_to_mat(box_pose.pose)
        box_ranges = [[-box_dims.x / 2, -box_dims.y / 2, -box_dims.z / 2],
                      [box_dims.x / 2, box_dims.y / 2, box_dims.z / 2]]

        self.draw_functions.draw_rviz_box(box_mat, box_ranges, '/fixed',
                                          ns='bounding box',
                                          color=[0, 0, 1], opaque=0.25, duration=60)

        # call the pickup service
        res = self.pickup(graspable_object, self.object.graspable_object_name, object_name)

        if res == 0: #correctly picked up
        #TODO: set up place_location
            initial_pose = PoseStamped()
            initial_pose.header.stamp = rospy.get_rostime()
            initial_pose.header.frame_id = "/fixed"
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.1
            initial_pose.pose.position.z = 0.0
            q=transformations.quaternion_about_axis(0.0, (1,0,0))
            initial_pose.pose.orientation.x = q[0]
            initial_pose.pose.orientation.y = q[1]
            initial_pose.pose.orientation.z = q[2]
            initial_pose.pose.orientation.w = q[3]

            list_of_poses = self.compute_list_of_poses(initial_pose, graspable_object)

            self.place_object(graspable_object, self.object.graspable_object_name, object_name, list_of_poses)

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

        pickup_goal.ignore_collisions = True

        pickup_goal.arm_name = "right_arm"
        #pickup_goal.desired_approach_distance = 0.05
        #pickup_goal.min_approach_distance = 0.02

        direction = Vector3Stamped()
        direction.header.stamp = rospy.get_rostime()
        direction.header.frame_id = "/base_link";
        direction.vector.x = 0;
        direction.vector.y = 0;
        direction.vector.z = 1;
        pickup_goal.lift.direction = direction;
        #request a vertical lift of 15cm after grasping the object
        pickup_goal.lift.desired_distance = 0.05;
        pickup_goal.lift.min_distance = 0.03;
        #do not use tactile-based grasping or tactile-based lift
        pickup_goal.use_reactive_lift = True;
        pickup_goal.use_reactive_execution = True;

        pickup_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_pickup', PickupAction)
        pickup_client.wait_for_server()
        rospy.loginfo("Pickup server ready")

        pickup_client.send_goal(pickup_goal)
        #TODO: change this when using the robot
        pickup_client.wait_for_result(timeout=rospy.Duration.from_sec(3600.0))
        loginfo("Got Pickup results")
        self.pickup_result = pickup_client.get_result()

        #print "HELLO: "+str(self.pickup_result)
        if pickup_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("The pickup action has failed: " + str(self.pickup_result.manipulation_result.value) )
            QMessageBox.warning(self, "Warning",
                    "Pickup action failed: "+str(self.pickup_result.manipulation_result.value))
            return -1

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
        direction.header.frame_id = "/base_link"
        direction.vector.x = 0
        direction.vector.y = 0
        direction.vector.z = -1
        place_goal.approach.direction = direction
        place_goal.approach.min_distance = 0.05
        place_goal.approach.desired_distance = 0.2
        #request a vertical put down motion of 10cm before placing the object
        place_goal.desired_retreat_distance = 0.1
        place_goal.min_retreat_distance = 0.05
        #we are not using tactile based placing
        place_goal.use_reactive_place = False

        place_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_place', PlaceAction)
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

    def compute_list_of_poses(self, initial_pose, graspable_object, rect_w=0.10, rect_h=0.10, resolution=0.02):
        '''
        Computes a list of possible poses in a rectangle of 2*rect_w by 2*rect_h, with the given resolution.
        In our case, rect_w is along the x axis, and rect_h along the y_axis.
        '''
        list_of_poses = []

        current_pose = PoseStamped()
        current_pose.header.stamp = rospy.get_rostime()
        current_pose.header.frame_id = "/fixed"
        current_pose.pose.position.x = initial_pose.pose.position.x
        current_pose.pose.position.y = initial_pose.pose.position.y
        current_pose.pose.position.z = initial_pose.pose.position.z
        current_pose.pose.orientation.x = initial_pose.pose.orientation.x
        current_pose.pose.orientation.y = initial_pose.pose.orientation.y
        current_pose.pose.orientation.z = initial_pose.pose.orientation.z
        current_pose.pose.orientation.w = initial_pose.pose.orientation.w
        rospy.loginfo( " placing object at: "+str(current_pose)  )

        list_of_poses.append(current_pose)

        self.draw_place_area(list_of_poses, graspable_object)

        return list_of_poses

    def draw_place_area(self, list_of_poses, graspable_object):
        '''
        Displays all the possible placing locations that are going to be tried.
        '''
        #try:
        (trans_palm,rot_palm) = self.listener.lookupTransform('/fixed', '/palm', rospy.Time(0))
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
            self.draw_functions.draw_rviz_box(mat, [.01,.01,.01], frame='/fixed', ns='place_'+str(index),
                                              id=1000+index, duration = 90, color=[0.5,0.5,0.0], opaque=1.0 )

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
    A map of TableObject is stored in HandleGuiManipulation.
    """
    def __init__(self):
        self.graspable_object      = None
        self.graspable_object_name = None
        self.model_description     = None


class Model(object):
    def __init__(self):
        self.name = None


class HandleGuiManipulation(QObject):
    """The main GUI dock window"""
    def __init__(self, parent, plugin_context):
        super(HandleGuiManipulation, self).__init__(parent)

        self.setObjectName('HandleGuiManipulation')
        self.srv_wait_time = 0.5
        # Set by call to process_collision_map()
        self.collision_support_surface_name = None
        self.raw_objects            = None
        self.found_objects          = {}
        self.unknown_object_counter = 1 #starts at 1 as it's only used for display
        self.service_tabletop_collision_map   = None
        self.service_db_get_model_description = None
        self.service_object_detector          = None
        self.existing_model_names = []
        self.bbox = (PoseStamped(),Vector3())
        self.geom_planner_rot_z=20.0
        self.geom_planner_trans_z=0.01
        self.grasp_syn_id=8
        self.tflistener = tf.TransformListener()
        self.draw_functions = draw_functions.DrawFunctions('grasp_markers')

        self.graspsLabel = ["Large diameter","Small diameter","Medium Wrap","Abducted Thumb","Light tool","Prismatic 4 fingers","Prismatic 3 fingers","Prismatic 2 fingers","Palmar Pinch","Power Disk","Power Sphere","Precision Disk","Precision Sphere","Tripod","Fixed Hook","Lateral","Index Finger Extension","Extension type","Distal type","Writing Tripod","Tripod Variation","Parallel Extension","Adduction Grip","Tip Pinch","Lateral Tripod","Sphere 4 fingers","Quadpd","Sphere 3 Fingers","Stick","Palmar","Ring","Ventral","Inferior Pincer","Flat","Halt","Empty","Inactive","Fist","Cup","Cage"]
        self.grasp_images = []
        
        self_dir        = os.path.dirname(os.path.realpath(__file__));
        self.config_dir = os.path.join(self_dir, '../config')
        self.ui_dir     = os.path.join(self_dir, '../ui')
        
        self.init_images()

        # UI setup
        ui_file = os.path.join(self.ui_dir, 'HandleGuiManipulation.ui')

        main_window = plugin_context.main_window()
        self.win = QDockWidget(main_window)
        loadUi(ui_file, self.win)
        if plugin_context.serial_number() > 1:
            self.win.setWindowTitle(
                    self.win.windowTitle() + (' (%d)' % plugin_context.serial_number()) )
        main_window.addDockWidget(Qt.RightDockWidgetArea, self.win)

        # trigger deleteLater for plugin when win is closed
        self.win.installEventFilter(self)

        # initialize grasp combo boxes with grasp names
        for i in range(len(self.graspsLabel)):
            self.win.syn_combobox.addItem(str(i)+":"+self.graspsLabel[i])
        
        self_dir        = os.path.dirname(os.path.realpath(__file__));

        # Bind button clicks
        self.win.btn_detect_objects.pressed.connect(self.detect_objects)
        self.win.btn_detect_objects.setEnabled(False);
        #self.win.btn_detect_objects.pressed.connect(self.set_tracker)
        self.win.btn_add_collision_map.pressed.connect(self.add_to_collision_map)
        self.win.btn_add_collision_map.setEnabled(False)
        self.win.btn_test_grasp.pressed.connect(self.test_grasp)
        self.win.btn_test_grasp.setEnabled(False)
        self.win.btn_test_grasp_quality.pressed.connect(self.test_grasp_quality)
        self.win.btn_test_grasp_quality.setEnabled(False)
        #self.win.btn_geom_planner.pressed.connect(self.geom_planner_exec)
        self.win.btn_geom_planner_rot_z20.pressed.connect(self.geom_planner_exec_rot_z)
        self.win.btn_geom_planner_rot_zm20.pressed.connect(self.geom_planner_exec_rot_zm)
        self.win.btn_geom_planner_trans_z10.pressed.connect(self.geom_planner_exec_trans_z)    
        self.win.btn_geom_planner_rot_z20.setEnabled(False)
        self.win.btn_geom_planner_rot_zm20.setEnabled(False)
        self.win.btn_geom_planner_trans_z10.setEnabled(False)
        self.win.btn_reconstruct_objects.pressed.connect(self.reconstruct_objects)
        self.win.btn_reconstruct_objects.setEnabled(False)
        self.win.btn_gen_grasp.pressed.connect(self.gen_grasp)
        self.win.btn_gen_grasp.setEnabled(False)
        self.win.btn_gen_grasp_seq.pressed.connect(self.gen_grasp_seq)
        self.win.btn_gen_grasp_seq.setEnabled(False)
        self.win.btn_rest_pose.pressed.connect(self.grasp_syn_rest_pose)
        self.win.btn_rest_pose.setEnabled(False)
        self.win.btn_syn_grasp.pressed.connect(self.grasp_syn_exec_pose)
        self.win.btn_syn_grasp.setEnabled(False)
        self.win.btn_syn_grasp_cancel.pressed.connect(self.grasp_syn_cancel)
        self.win.btn_syn_grasp_cancel.setEnabled(False)
        self.win.btn_retrack_obj.pressed.connect(self.retrack_obj)
        self.win.btn_retrack_obj.setEnabled(False)
        self.win.btn_adjust_track.pressed.connect(self.adjust_track)
        self.win.btn_adjust_track.setEnabled(False)
        self.win.btn_bias_sensors.pressed.connect(self.bias_sensors)
        self.win.btn_bias_sensors.setEnabled(False)
        self.win.btn_hold_firmly.pressed.connect(self.hold_more_firmly)
        self.win.btn_hold_firmly.setEnabled(False)
        self.win.btn_lift.pressed.connect(self.lift)
        self.win.btn_lift.setEnabled(False)
        self.win.btn_reset_lift.pressed.connect(self.relift)
        self.win.btn_reset_lift.setEnabled(False)

        self.win.syn_combobox.currentIndexChanged.connect(self.on_syn_combo_change)
        self.win.syn_combobox.setCurrentIndex(self.grasp_syn_id)
        rospy.loginfo("init services")
        self.init_services()
        rospy.loginfo("init_data")
        self.init_data()
        rospy.loginfo("init done")

    def init_services(self):
        self.service_tabletop_collision_map=None
        self.service_get_model_by_acquisition=None
        self.service_db_get_model_description =None
        self.service_get_object_pose=None
        self.service_get_central_object=None
        self.service_set_tracked_object=None
        self.service_bias_sensors=None
        self.service_get_model_list=None
        self.service_geom_planner_exec=None
        
        rospy.loginfo("Connecting to services")
        """Service setup"""
        # ManipStack nodes
        srvname = '/tabletop_collision_map_processing/tabletop_collision_map_processing'
        try:
            rospy.loginfo("Wait for tabletop node")
            rospy.wait_for_service(srvname,self.srv_wait_time)
            self.service_tabletop_collision_map = rospy.ServiceProxy(srvname, TabletopCollisionMapProcessing)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")

        # UC3M DB
        srvname = '/uc3m_hdb/objects_database_node/get_model_description'
        try:
            rospy.loginfo("Wait for UC3M database node")
            rospy.wait_for_service(srvname,self.srv_wait_time)
            self.service_db_get_model_description = rospy.ServiceProxy(srvname, GetModelDescription)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")

        # UC3M object tracker
        srvname = '/uc3m_hdb/get_object_pose'
        try:
            rospy.loginfo("Wait for UC3M tracker node")
            rospy.wait_for_service(srvname,self.srv_wait_time)
            self.service_get_object_pose = rospy.ServiceProxy(srvname, GetObjectPose)
            self.win.btn_detect_objects.setEnabled(True);
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")

        # UC3M object detect
        srvname = '/uc3m_hdb/get_central_object_on_table'
        try:
            rospy.loginfo("Wait for UC3M detect node")
            rospy.wait_for_service(srvname,self.srv_wait_time)
            self.service_get_central_object = rospy.ServiceProxy(srvname, GetCentralObjectOnTable)
            self.win.btn_reconstruct_objects.setEnabled(True)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")

        # UC3M list access via standard DB (be careful to use correct DB)
        srvname = '/uc3m_hdb/objects_database_node/get_model_list'
        try:
            rospy.loginfo("Wait for UC3M standard database node")
            rospy.wait_for_service(srvname,self.srv_wait_time)
            self.service_get_model_list = rospy.ServiceProxy(srvname, GetModelList)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")
            
        # UC3M obj list access via their DB 
        srvname = '/uc3m_hdb/get_list_by_acquisition_method'
        try:
            rospy.loginfo("Wait for UC3M get model by acquisition service")
            rospy.wait_for_service(srvname,self.srv_wait_time)
            self.service_get_model_by_acquisition = rospy.ServiceProxy(srvname, getModelbyAcquisition)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")
            
        # UPMC fake tracker
        srvname = '/fake_tracker/set_tracked_object'
        try:
            rospy.loginfo("Wait for UPMC tracker")
            rospy.wait_for_service(srvname,self.srv_wait_time)
            self.service_set_tracked_object = rospy.ServiceProxy(srvname, SetTrackedObject)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")
            
        # UPMC geom planner
        srvname = '/execute_in_hand_mvt'
        try:
            rospy.loginfo("Wait for UPMC geom planner")
            rospy.wait_for_service(srvname,self.srv_wait_time)
            self.service_geom_planner_exec = rospy.ServiceProxy(srvname, execute_in_hand_mvt)
            self.win.btn_geom_planner.setEnabled(True)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")
            
        # UPMC grasp generator
        srvname = '/rotation_symetric_grasp_generator'
        try:
            rospy.loginfo("Wait for UPMC rot grasp generator")
            rospy.wait_for_service(srvname,1)
            self.service_gen_rot_grasp = rospy.ServiceProxy(srvname, GraspRotation)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")
           
        # KCL 
        srvname = '/nano17ft/sensor_bias'
        try:
            rospy.loginfo("Wait for KCL sensors")
            rospy.wait_for_service(srvname,self.srv_wait_time)
            self.service_bias_sensors = rospy.ServiceProxy(srvname, KCL_Sensor_Bias)
            self.win.btn_bias_sensors.setEnabled(True)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")
            
        # KCL rect
        srvname = '/rectify_pose_tabletop'
        try:
            rospy.loginfo("Wait for KCL pose rect")
            rospy.wait_for_service(srvname,1)
            self.service_rectify_pose = rospy.ServiceProxy(srvname, KCL_PoseRectifyEReq)
            #self.service_rectify_pose = rospy.ServiceProxy(srvname, Empty)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")    
            
        self.syn_client = actionlib.SimpleActionClient('synergy_grasp', SynergyGraspAction)
        # TODO: Test availability of some topics or such to activate the syn grasp button

    def init_data(self):
        if self.service_get_model_by_acquisition!=None:
            model_ids1=self.query_object_list("handle_uc3m_single_view")
            model_ids2=self.query_object_list("handle_manual")
            existing_model_ids=[]
            existing_model_ids+=model_ids1
            existing_model_ids+=model_ids2
            self.existing_model_names=[]
            for model_id in existing_model_ids:
                mymodel=self.get_object_name(model_id)
                self.existing_model_names.append(mymodel.name)  

    # pre-load all config images
    def init_images(self):
        for i in range(len(self.graspsLabel)):
            if(i<10):
                image_path='../images/config_0'+str(i+1)+'.png'
            else:
                image_path='../images/config_'+str(i+1)+'.png'
            self.grasp_images.append(QPixmap(os.path.join(os.path.dirname(os.path.realpath(__file__)), image_path)))


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

    def on_syn_combo_change(self):
        # get current combo box index
        self.grasp_syn_id=self.win.syn_combobox.currentIndex()
        # get image related to this index
        myimage=self.grasp_images[self.grasp_syn_id]
        # set the image
        self.win.syn_image.setPixmap(myimage.copy(0,0,170,170))


    def query_object_list(self,aq_method):
        # handle_uc3m_single_view or handle_manual
        request=getModelbyAcquisitionRequest()
        request.acquisition_method=aq_method
        try:
            response=self.service_get_model_by_acquisition(request)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
        return response.model_ids

    def createPlatformTable(self):
        ptab=Table()
        ptab.pose.header.frame_id="/world"
        ptab.pose.header.stamp=rospy.Time.now()
        ptab.pose.pose=Pose(Point(0.25,0.25,1.0),Quaternion(0,0,0,1))
        ptab.x_min=0.0
        ptab.x_max=0.5
        ptab.y_min=0.0
        ptab.y_max=0.5
        return ptab

    def construct_object(self,new_object_name):
        request=GetCentralObjectOnTableRequest()
        request.path=new_object_name
        #ignore response as we will track the object anyway afterwards
        self.service_get_central_object(request) 
 
    def call_find_cluster_bounding_box(self, cluster):
        
        req = FindClusterBoundingBoxRequest()
        req.cluster = cluster
        service_name = "find_cluster_bounding_box"
        rospy.loginfo("waiting for find_cluster_bounding_box service")
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found")
        serv = rospy.ServiceProxy(service_name, FindClusterBoundingBox)
        try:
            myres = serv(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling find_cluster_bounding_box: %s" % e)
            return 0
        if not myres.error_code:
            self.tflistener.waitForTransform("/world", "/camera_rgb_frame", myres.pose.header.stamp, rospy.Duration.from_sec(4.0))
            newpose=self.tflistener.transformPose("/world",myres.pose)
            # store new position (but not orientation as the axis is z up in kinect frame)
            myres.pose.pose.position=newpose.pose.position
            box_ranges = [[-myres.box_dims.x / 2, -myres.box_dims.y / 2, -myres.box_dims.z / 2],
                      [myres.box_dims.x / 2, myres.box_dims.y / 2, myres.box_dims.z / 2]]
            box_mat = pose_to_mat(myres.pose.pose)
            self.draw_functions.draw_rviz_box(box_mat, box_ranges, '/world',
                                          ns='bounding box',
                                          color=[0, 0, 1], opaque=0.25, duration=60)
            return (myres.pose, myres.box_dims)
        else:
            return (None, None)
    
    def test_grasp(self):
        print "test grasp"
        
    def grasp_syn_set_ID(self):
        self.grasp_syn_id=1.0
            
    def grasp_syn_exec(self,goal):
        # Waits until the action server has started up and started
        # listening for goals.
        self.syn_client.wait_for_server()
        print self.grasp_syn_id
        # Sends the goal to the action server.
        self.syn_client.send_goal(goal)  
        # Waits for the server to finish performing the 
        #actionfinished=syn_client.wait_for_result(rospy.Duration.from_sec(30.0))
        
        #if(actionfinished):
        #    rospy.loginfo("action finished")
        #else:
        #    rospy.loginfo("action timedout")
        # Prints out the result of executing the action
        #return self.syn_client.get_result() 
    
    def grasp_syn_rest_pose(self):
        print "Go to rest pose"
         # Creates a goal to send to the action server.
        goal = SynergyGraspGoal()
        goal.grasp_class_name="Pregrasp"
        goal.grasp_class_number=33
        self.grasp_syn_exec(goal)
        
    def grasp_syn_exec_pose(self):
        print "Go to pose"
         # Creates a goal to send to the action server.
        goal = SynergyGraspGoal()
        goal.grasp_class_name="Palmar pinch"
        goal.grasp_class_number=8
        
        #random data because not used yet
        goal.grasp_parameters.append(0.0)
        goal.grasp_parameters.append(0.5)
        goal.grasp_parameters.append(1.0)
        goal.grasp_parameters.append(0.5)
        goal.grasp_parameters.append(0.0)
        self.grasp_syn_exec(goal)
            
    def grasp_syn_cancel(self):
        print "Cancel current grasp synergy"
        cancel_goal=GoalID()
        cancelpub = rospy.Publisher("/synergy_grasp/cancel", GoalID)
        cancelpub.publish(cancel_goal)
        
    def test_grasp_quality(self):
        print "testing grasp quality"
        
    def lift(self):
        print "Lift !"
        
    def relift(self):
        print "Relift !"
        
    def hold_more_firmly(self):
        print "Hold more firmly !"
        
    def geom_planner_exec(self):
        print "Geom Planner execution"
        self.obj_position.x=float(self.win.geom_posX.toPlainText())
        self.obj_position.y=float(self.win.geom_posY.toPlainText())
        print "Obj Pose:",self.obj_position.x," ,",self.obj_position.y
        self.set_tracker()
        myrequest=execute_in_hand_mvtRequest()
        myrequest.desired_in_hand_mvt.model_id=8881
        #myrequest.desired_in_hand_mvt.movement=Twist(Vector3(0,0,0),Vector3(0,0,self.geom_planner_rot_z))
        myrequest.desired_in_hand_mvt.movement=Twist(Vector3(0,0,self.geom_planner_trans_z),Vector3(0,0,self.geom_planner_rot_z))
        if(self.win.geom_checkbox.isChecked()):
            mysteps=1
        else:
            mysteps=2
        myrequest.desired_in_hand_mvt.steps=mysteps
        self.service_geom_planner_exec(myrequest)        
        
    def geom_planner_exec_rot_z(self):
        self.geom_planner_rot_z=20.0
        self.geom_planner_trans_z=0
        self.geom_planner_exec()
        
    def geom_planner_exec_rot_zm(self):
        self.geom_planner_rot_z=-20.0
        self.geom_planner_trans_z=0
        self.geom_planner_exec()
        
    def geom_planner_exec_trans_z(self):
        self.geom_planner_trans_z=0.01
        self.geom_planner_rot_z=0.0
        self.geom_planner_exec()
        
    def gen_grasp(self):
        print "gen grasp !"
        myrequest=GraspRotationRequest()
        mygrasp=Grasp()
        mygrasp.pre_grasp_posture=JointState()
        
        mygrasp.pre_grasp_posture.name=['FFJ4', 'FFJ3', 'FFJ0', 'MFJ4', 'MFJ3', 'MFJ0', 'RFJ4', 'RFJ3', 'RFJ0', 'LFJ5', 'LFJ4', 'LFJ3', 'LFJ0', 'THJ5', 'THJ4', 'THJ3', 'THJ2', 'THJ1']
        mygrasp.pre_grasp_posture.position=[-0.436, 0.005, 0.801, 0.0, 0.015, 0.791, -0.436, 0.005, 0.801, 0.0 , -0.436, 0.0, 0.811, 0.0, 1.309, -0.262, -0.524, 0.248]
        mygrasp.grasp_posture=mygrasp.pre_grasp_posture
        mygrasp.grasp_pose=Pose(Point(-0.0645,-0.055,0.19),Quaternion(0.579,0.579,0.405,0.405))
        mygrasp.success_probability=1.0
        mygrasp.cluster_rep= True;
        mygrasp.desired_approach_distance=0.2
        mygrasp.min_approach_distance=0.1
        myrequest.input_grasps.grasps.append(mygrasp)
        myrequest.input_grasps.error_code.value=0
        self.grasp_list=self.service_gen_rot_grasp(myrequest)       #GraspPlanningResult message as a response

    def gen_grasp_seq(self):
        print "gen grasp seq!"
        
    def adjust_track(self):
        print "adjust_track !"
        '''
        myrequest=KCL_PoseRectifyRequest()
        if self.pcloud_palm!=[]:
            myrequest.point_cloud_in=self.pcloud_palm
            translation=self.service_rectify_pose(myrequest)
            self.obj_position.x+=translation.x
            self.obj_position.y+=translation.y
            self.obj_position.z+=translation.z
            self.set_tracker()
        '''
        self.service_rectify_pose()
    
    
    def retrack_obj(self):
        print "retrack obj !"
        if self.detect_objects():
            #transform pointcloud into 
            pcloud=self.raw_objects.clusters[0]
            self.tflistener.waitForTransform("/palm", "/camera_rgb_frame", pcloud.header.stamp, rospy.Duration.from_sec(1.0))
            self.pcloud_palm=self.tflistener.transformPointCloud("/palm",pcloud)
            print self.pcloud_palm.header.frame_id
            self.pcloud_palm.header.frame_id="/palm"
        else:
            self.pcloud_palm=PointCloud()
        
    def bias_sensors(self):
        print "Bias sensors !"
        myrequest=KCL_Sensor_BiasRequest()
        myrequest.id=[1,2,3,4,5]
        self.service_bias_sensors(myrequest)        
        
    def reconstruct_objects(self):
        print "Reconstruct !"
        #print self.win.reconstruct_objects_name.text()
        print self.win.textEdit.toPlainText()
        myrequest=GetCentralObjectOnTableRequest()
        myrequest.path(str(self.win.textEdit.toPlainText()))
        self.service_get_central_object(myrequest)

    def set_tracker(self):
        if self.service_set_tracked_object!=None:
            myrequest=SetTrackedObjectRequest()
            myrequest.tracked_object.model_id=8881
            myrequest.tracked_object.pose=PoseStamped()
            myrequest.tracked_object.pose.header.frame_id="world"
            myrequest.tracked_object.pose.header.stamp=rospy.Time.now()
            myrequest.tracked_object.pose.pose=Pose(self.obj_position,Quaternion(0,0,0,1))
            myrequest.tracked_object.confidence=1
            myrequest.tracked_object.detector_name="uc3m"
            self.win.geom_posX.setText(str(self.obj_position.x))
            self.win.geom_posY.setText(str(self.obj_position.y))
            self.service_set_tracked_object(myrequest)

    def detect_objects(self):
        self.found_objects.clear()
        self.win.contents.setCursor(Qt.WaitCursor)
        detect_obj_req=GetObjectPoseRequest()
        self.raw_objects=TabletopDetectionResult()
        self.raw_objects.table=self.createPlatformTable()
        
        try:
            myresponse=self.service_get_object_pose(detect_obj_req)
            #print "detect",myresponse
            self.raw_objects.clusters.append(myresponse.object_point_cloud)
            self.raw_objects.cluster_model_indices.append(0)
            self.raw_objects.result=TabletopDetectionResult.SUCCESS
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
            return False

        if self.raw_objects != None:
            self.bbox=self.call_find_cluster_bounding_box(self.raw_objects.clusters[0])
            #print self.bbox
            self.obj_position=self.bbox[0].pose.position
            self.obj_position.z-=0.05
            self.set_tracker()
            self.win.btn_add_collision_map.setEnabled(True)
        else:
            return False
        # TODO: Should really do this with SIGNALs and SLOTs.
        self.win.contents.setCursor(Qt.ArrowCursor)
        return True
        
    def add_to_collision_map(self):
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


    def process_collision_map(self):
        res = 0
        try:
            #Args: detection_result reset_collision_models reset_attached_models desired_frame
            res = self.service_tabletop_collision_map(self.raw_objects, True, True, "/fixed")
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

