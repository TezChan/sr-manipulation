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

import roslib; roslib.load_manifest('sr_object_manipulation_smach')
import rospy

import smach
import smach_ros

from state_machines_library.full_object_manipulation import SrFullObjectManipulationStateMachine


def main():
    """
    """
    rospy.init_node('sr_object_manipulation_smach')
    sm = SrFullObjectManipulationStateMachine()
    sm.state_machine.execute()

    rospy.spin()

if __name__ == '__main__':
    main()            
