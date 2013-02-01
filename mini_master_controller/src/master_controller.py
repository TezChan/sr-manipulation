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


import roslib; roslib.load_manifest('mini_master_controller')
import rospy
import actionlib

from handle_demo_step_actions.msg import MasterControllerAction, MasterControllerResult, MasterControllerFeedback, ObjectRecognitionAction, ObjectSelectionAction, PickObjectAction, PlacePositionSelectionAction, PlaceObjectAction, ObjectRecognitionGoal, ObjectSelectionGoal, PickObjectGoal, PlacePositionSelectionGoal, PlaceObjectGoal  
from handle_demo_step_actions.srv import NextStepSelection

class StepInformation(object):
    def __init__(self, actionlib_client):
        self.client = actionlib_client
        self.result = None

class MasterController(object):
    def __init__(self):
        #Contains the list of step names contained in the goal received
        self.step_name_list = None
        #A dictionary to be able to find the order index of a step given the step_name
        self.step_order_dict = {}
        #dictionary containing the actionlib clients for the steps
        self.step_clients = {}
        
        self.service_next_step = None
        
        self.server = actionlib.SimpleActionServer('master_controller', MasterControllerAction, self.execute, False)
        #We register a callback function for the Cancel topic
        self.server.register_preempt_callback(self.cancel_cb)
        self.server.start()
        rospy.loginfo("Master Controller: MC server ready")
        
    def init_services(self):
        #Creates a ServiceProxy to call the service to select the next step
        srvname = 'decision_services/next_step'
        rospy.loginfo("Waiting for service " + srvname)
        rospy.wait_for_service(srvname)
        self.service_next_step = rospy.ServiceProxy(srvname, NextStepSelection)
    
    def execute(self, goal):
        if len(goal.steps_to_execute) == 0:
            rospy.logwarn("Empty execution steps list")
            self.server.set_aborted(MasterControllerResult(1))
            return
        self.step_order_dict.clear()
        
        self.step_name_list = goal.steps_to_execute
        
        #create_step_actionlib_clients returns 1 if there's an error, 0 if it succeeds
        if self.create_step_actionlib_clients(self.step_name_list):
            rospy.logerr("Error in some of the actionlib clients creation. Aborting goal.")
            self.server.set_aborted(MasterControllerResult(1))
            return
        
        rospy.loginfo("Actionlib clients created")
        
        exit_condition = False
        
        if goal.first_step in self.step_order_dict:
            i = self.step_order_dict[goal.first_step]
            rospy.loginfo("First step received in goal: " + goal.first_step)
        else:
            rospy.logerr("Unknown first_step name: " + goal.first_step + ". Starting from step 0")
            #We start in the first step of the list (0)
            i = 0
        
        while (not exit_condition) and (not rospy.is_shutdown()):
            if self.server.is_preempt_requested():
                #If the user cancels we'll send the first step as a feedback
                self.server.publish_feedback(MasterControllerFeedback(self.step_name_list[0]))
                self.server.set_preempted(MasterControllerResult(1))
                return
             
            if rospy.get_param('master_controller/autorun', 1) == 0:
                #if not in autorun
                #Call a service to see what's next
                next_step_name = None
                
                if not self.service_next_step:
                    self.init_services()
                    
                try:
                    next_step_srv_response = self.service_next_step(self.step_name_list[i])
                    next_step_name = next_step_srv_response.next_step
                except rospy.ServiceException, e:
                    print "Service did not process request: %s" % str(e)
                    self.server.set_aborted(MasterControllerResult(1))
                    return
                
                if next_step_name == "next":
                    if i < (len(self.step_name_list) - 1):
                        i = i + 1
                    else:
                        #Result 0 means that it finished succesfully
                        self.server.set_succeeded(MasterControllerResult(0))
                        return
                else:  #if we've receive the name of a step as an answer
                    if next_step_name in self.step_order_dict:
                        i = self.step_order_dict[next_step_name]
                    else:
                        rospy.logerr("Unknown step name: " + next_step_name)
                        continue
            
            self.server.publish_feedback(MasterControllerFeedback(self.step_name_list[i]))
            self.run_step(self.step_name_list[i])
            
            if rospy.get_param('master_controller/autorun', 1) != 0:
                #in autorun we just run the next step
                if i < (len(self.step_name_list) - 1):
                    i = i + 1
                else:
                    #Result 0 means that it finished succesfully
                    self.server.set_succeeded(MasterControllerResult(0))
                    return
            
    
    def create_step_actionlib_clients(self, step_name_list):
        for (i, step_name) in enumerate(step_name_list):
            rospy.loginfo("Creating actionlib client for step: " + step_name)
            self.step_order_dict[step_name] = i
            if step_name in self.step_clients:
                #If this step already exists in the step_clients dictionary we will not create a new client (as it's not necessary)
                #This is useful as well because it allows us to keep the latest result for this step, 
                #which will be necessary if we want to start the execution from a step other than the first.
                rospy.loginfo("Using existing actionlib client for step: " + step_name)
                continue
            
            client = None
            
            if step_name == 'object_recognition':
                server_name = 'recognise_objects'
                client = actionlib.SimpleActionClient(server_name, ObjectRecognitionAction)
            elif step_name == 'object_selection':
                server_name = 'select_object'
                client = actionlib.SimpleActionClient(server_name, ObjectSelectionAction)
            elif step_name == 'pick_object':
                server_name = 'pick_object'
                client = actionlib.SimpleActionClient(server_name, PickObjectAction)
            elif step_name == 'place_position_selection':
                server_name = 'select_place_position'
                client = actionlib.SimpleActionClient(server_name, PlacePositionSelectionAction)
            elif step_name == 'place_object':
                server_name = 'place_object'
                client = actionlib.SimpleActionClient(server_name, PlaceObjectAction)
            else:
                rospy.logerr("Unknown step name: " + step_name + ". Please add this step to the MC implementation if the name is correct.")
                return 1
            
            if not client:
                rospy.logerr("Error creating SimpleActionClient: " + server_name)
                return 1
            rospy.loginfo("Waiting for server: " + server_name)    
            client.wait_for_server()
            self.step_clients[step_name] = StepInformation(client)
            
        return 0

    def run_step(self, step_name):
        goal = None
        step_info = self.step_clients[step_name]
        
        if step_name == 'object_recognition':
            goal = ObjectRecognitionGoal()
        elif step_name == 'object_selection':
            if self.step_clients['object_recognition'].result != None:
                goal = ObjectSelectionGoal()
                goal.objects = self.step_clients['object_recognition'].result.objects
        elif step_name == 'pick_object':
            if (self.step_clients['object_selection'].result != None) and (self.step_clients['object_recognition'].result != None):
                goal = PickObjectGoal()
                goal.object_to_pick = self.step_clients['object_selection'].result.selected_object
                goal.collision_support_surface_name = self.step_clients['object_recognition'].result.collision_support_surface_name
        elif step_name == 'place_position_selection':
            if self.step_clients['pick_object'].result != None:
                goal = PlacePositionSelectionGoal()
                goal.initial_pose_of_the_object = self.step_clients['pick_object'].result.initial_pose
        elif step_name == 'place_object':
            if (self.step_clients['place_position_selection'].result != None) and (self.step_clients['object_selection'].result != None) \
                and (self.step_clients['pick_object'].result != None) and (self.step_clients['object_recognition'].result != None):
                goal = PlaceObjectGoal()
                goal.place_pose = self.step_clients['place_position_selection'].result.selected_place_pose
                goal.object_to_place = self.step_clients['object_selection'].result.selected_object
                goal.executed_grasp = self.step_clients['pick_object'].result.executed_grasp
                goal.collision_support_surface_name = self.step_clients['object_recognition'].result.collision_support_surface_name
        
        if goal == None:
            rospy.logerr("No valid goal for step: " + step_name + ". Probably invalid result for previous step or previous step not run.")
            return
        
        step_info.client.send_goal(goal)
        step_info.client.wait_for_result()
        step_info.result = step_info.client.get_result()
            
    def cancel_cb(self):
        #We cancel the goals for all the steps (it only really affects the currently running step)
        for step_name in self.step_name_list:
            self.step_clients[step_name].client.cancel_goal()        

if __name__ == '__main__':
    rospy.init_node('master_controller')
    server = MasterController()
    rospy.spin()
