from __future__ import division
import os
import math

import roslib; roslib.load_manifest('smr_controllers_gui')
import rospy
from rospy import loginfo, logerr, logdebug
from std_msgs.msg import Float64

from rosgui.QtBindingHelper import loadUi
import QtGui
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint, SIGNAL
from QtGui import *

from smr_fingertip_controllers_msgs.srv import smrFtControlModeSrv, smrFtControlModeSrvRequest, smrFtControlModeSrvResponse
from smr_fingertip_controllers_msgs.srv import smrFtControlParamSrv, smrFtControlParamSrvRequest, smrFtControlParamSrvResponse


class SmrControllersGui(QObject):
    """The main GUI dock window"""
    def __init__(self, parent, plugin_context):
        super(SmrControllersGui, self).__init__(parent)

        self.setObjectName('SmrControllersGui')
        # Set by call to process_collision_map()

        self_dir        = os.path.dirname(os.path.realpath(__file__));
        self.ui_dir     = os.path.join(self_dir, '../ui')

        # UI setup
        ui_file = os.path.join(self.ui_dir, 'SmrControllersGui.ui')

        main_window = plugin_context.main_window()
        self.win = QDockWidget(main_window)
        loadUi(ui_file, self.win)
        if plugin_context.serial_number() > 1:
            self.win.setWindowTitle(
                    self.win.windowTitle() + (' (%d)' % plugin_context.serial_number()) )
        main_window.addDockWidget(Qt.RightDockWidgetArea, self.win)

        # trigger deleteLater for plugin when win is closed
        self.win.installEventFilter(self)
        self.win.slider_max_forceFF.setMinimum(3)
        self.win.slider_max_forceFF.setMaximum(30)
        self.win.slider_radiusFF.setMinimum(1)
        self.win.slider_radiusFF.setMaximum(100)
        self.win.slider_max_forceTH.setMinimum(3)
        self.win.slider_max_forceTH.setMaximum(30)
        self.win.slider_radiusTH.setMinimum(1)
        self.win.slider_radiusTH.setMaximum(100)

        # Bind button clicks
        #self.win.btn_detect_objects.pressed.connect(self.detect_objects)
        self.win.btn_setFF.pressed.connect(self.set_FFsettings)
        self.win.checkFF.stateChanged.connect(self.activate_FFcontroller)
        self.win.slider_max_forceFF.valueChanged.connect(self.forceFF_changed)
        self.win.slider_radiusFF.valueChanged.connect(self.radiusFF_changed)
        self.win.btn_setTH.pressed.connect(self.set_THsettings)
        self.win.checkTH.stateChanged.connect(self.activate_THcontroller)
        self.win.slider_max_forceTH.valueChanged.connect(self.forceTH_changed)
        self.win.slider_radiusTH.valueChanged.connect(self.radiusTH_changed)
        
        rospy.loginfo("init services")
        self.init_services()
        rospy.loginfo("init done")

    def init_services(self):
        self.service_tabletop_collision_map=None
        self.service_get_model_by_acquisition=None
        
        rospy.loginfo("Connecting to services")
        """Service setup"""
        # ManipStack nodes
        '''
        srvname = '/tabletop_collision_map_processing/tabletop_collision_map_processing'
        try:
            rospy.loginfo("Wait for tabletop node")
            rospy.wait_for_service(srvname,1)
            self.service_tabletop_collision_map = rospy.ServiceProxy(srvname, TabletopCollisionMapProcessing)
            rospy.loginfo("OK")
        except:
            rospy.logerr("not found")
        '''
       
       
    def set_FFsettings(self):
        print "force:",self.FFforce," radius: ",self.FFradius
        
    def forceFF_changed(self):
        self.FFforce=self.win.slider_max_forceFF.value()/10.0
        self.win.text_max_forceFF.setText("Max Force: "+str(self.FFforce))
    def radiusFF_changed(self):
        self.FFradius=self.win.slider_radiusFF.value()/1000.0
        self.win.text_radiusFF.setText("Radius: "+str(self.FFradius))
        
    def activate_FFcontroller(self):
        print self.win.checkFF.isChecked()
        
    def set_THsettings(self):
        print "force:",self.THforce," radius: ",self.THradius
        
    def forceTH_changed(self):
        self.THforce=self.win.slider_max_forceTH.value()/10.0
        self.win.text_max_forceTH.setText("Max Force: "+str(self.THforce))
    def radiusTH_changed(self):
        self.THradius=self.win.slider_radiusTH.value()/1000.0
        self.win.text_radiusTH.setText("Radius: "+str(self.THradius))
        
    def activate_THcontroller(self):
        print self.win.checkTH.isChecked()
        
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

