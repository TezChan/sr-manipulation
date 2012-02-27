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

import roslib; roslib.load_manifest('sr_gui_manipulation')
import rospy
from rospy import loginfo

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint
from QtGui import QDockWidget, QShortcut, QMessageBox, QFrame, QHBoxLayout, QCheckBox, QLabel, QCursor, QColor, QMessageBox

class SrGuiManipulation(QObject):

    def __init__(self, parent, plugin_context):
        super(SrGuiManipulation, self).__init__(parent)
        self.setObjectName('SrGuiManipulation')

        ui_file = os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                '../ui/SrGuiManipulation.ui')

        main_window = plugin_context.main_window()
        self.win = QDockWidget(main_window)
        loadUi(ui_file, self.win)
        if plugin_context.serial_number() > 1:
            self.win.setWindowTitle(
                    self.win.windowTitle() + (' (%d)' % plugin_context.serial_number()) )
        main_window.addDockWidget(Qt.RightDockWidgetArea, self.win)

        # trigger deleteLater for plugin when win is closed
        self.win.installEventFilter(self)

        # Bind button clicks
        #self.win.btn_select_all.pressed.connect(self.on_select_all_pressed)

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

