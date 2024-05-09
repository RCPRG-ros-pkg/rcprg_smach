#!/usr/bin/env python
# encoding: utf8

import math
import random
import rospy
import smach
import smach_ros
import dynamic_reconfigure.client
import actionlib

from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from tiago_rosplan_sim.srv import RescueService
from TaskER.TaskER import TaskER
from rcprg_smach import smach_rcprg

from pl_nouns.dictionary_client import DisctionaryServiceClient
from ROSPlan_communication.ROSPlanExecInterface import ROSPlanExecInterface
from ROSPlan_communication.rosplan_manager import rosplan_manager
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback, CompletePlan

import os
import task_manager
import threading
import time
import subprocess
import multiprocessing
import psutil
import shlex

class ExecuteROSPlan(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, task_kb_id, destination):
        TaskER.BlockingState.__init__(self, input_keys=[], output_keys=['output_destination'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])
        self.task_kb_id = task_kb_id
        self.action_id = None
        self.sim_mode = sim_mode
        self.conversation_interface = conversation_interface
        self.destination = destination
        self.rm = rosplan_manager(task_id=self.task_kb_id, task_name='rescue')

        print("Calling Actinf execution")
        subp = subprocess.Popen(shlex.split("roslaunch tiago_rosplan_sim launch_actinf.launch"))
        p = psutil.Process(subp.pid)
        print("Waiting for Actinf to setup")
        rospy.sleep(2)


    def transition_function(self, userdata):
        userdata.output_destination = self.destination
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        rpl_ei = ROSPlanExecInterface(da_type="rescue", da_id = self.task_kb_id, goal=self.destination)
        rpl_ei.run_plan_parse_and_dispatch()

        if self.__shutdown__:
            # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
            self.rm.before_finished_shutdown()
            return 'shutdown'
        return 'ok'

class MoveToVictim(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, task_kb_id):
        TaskER.BlockingState.__init__(self, input_keys=['input_destination'], output_keys=['output_destination'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])
        self.task_kb_id = task_kb_id
        self.action_id = None
        self.task_failed = False
        self.action_succeeded = False
        self.rm = rosplan_manager(task_id=self.task_kb_id, task_name='clean')

    def action_feedback_subscriber(self):
        aft = "/rosplan_plan_dispatcher/action_feedback"
        self.af_sub = rospy.Subscriber(aft, ActionFeedback, self.action_feedback_cb)

    def action_feedback_cb(self, action_feedback):
        (action_id, action_status) = self.get_feedback_action_params(action_feedback)
        if action_status == 1:
            print("Action " + str(action_id) + " enabled. (status 1)")
        if action_status == 2:
            print("Action " + str(action_id) + " succeeded to goal state. (status 2) plan len != action id")
            self.af_sub.unregister()
            self.action_succeeded = True
        if action_status == 10:
            print("Action " + str(action_id) + " failed. (status 10)")
            self.task_failed = True

    def get_feedback_action_params(self, action_feedback):
        action_id = action_feedback.action_id
        plan_id = action_feedback.plan_id
        action_status = action_feedback.status
        return (action_id, action_status)

    def transition_function(self, userdata):
        self.destination = userdata.input_destination
        userdata.output_destination = userdata.input_destination
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        action_feedback = None
        self.action_feedback_subscriber()
        print("Inside MoveTo transition function")
        while True:
            print("Heading to the " + self.destination + " in progress...")
            if self.__shutdown__:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.rm.before_shutdown()
                return 'shutdown'

            if self.action_succeeded:
                self.af_sub.unregister()
                return 'ok'

            if self.task_failed:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                return 'shutdown'
            rospy.sleep(1)
    
class EndActionEvent(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, task_kb_id):
        TaskER.BlockingState.__init__(self, input_keys=['input_destination'], output_keys=['output_destination'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])
        self.task_kb_id = task_kb_id
        self.action_id = None
        self.plan_actions = None
        self.task_succeeded = False
        self.task_failed = False
        self.action_succeeded = False

        self.rm = rosplan_manager(task_id=self.task_kb_id, task_name="rescue")
    
    def end_event_action_service_handler(self, req):
        rospy.sleep(5)
        return 1

    def end_event_action_service(self):
        self.end_event_action_srv = rospy.Service('/rescue_service', RescueService, self.end_event_action_service_handler)

    def action_feedback_subscriber(self):
        aft = "/rosplan_plan_dispatcher/action_feedback"
        self.af_sub = rospy.Subscriber(aft, ActionFeedback, self.action_feedback_cb)

    def action_feedback_cb(self, action_feedback):
        (action_id, action_status) = self.get_feedback_action_params(action_feedback)
        # rospy.loginfo('{}: Executing state EndActionEvent')
        if action_status == 1:
            print("Action " + str(action_id) + " enabled. (status 1)")
        if action_status == 2 and int(action_id) != int(self.plan_length)-1:
            print("Action " + str(action_id) + " succeeded to goal state. (status 2) plan len != action id")
            self.action_succeeded = True
        if action_status == 10:
            print("Action " + str(action_id) + " failed. (status 10)")
            self.task_failed = True
        if action_status == 2 and int(action_id) == int(self.plan_length)-1:
            print("Action " + str(action_id) + " succeeded to goal state. (status 2)")
            self.task_succeeded = True
    
    def get_feedback_action_params(self, action_feedback):
        action_id = action_feedback.action_id
        plan_id = action_feedback.plan_id
        action_status = action_feedback.status
        return (action_id, action_status)
                    
    def transition_function(self, userdata):
        self.destination = userdata.input_destination
        userdata.output_destination = userdata.input_destination
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        self.plan_length = self.rm.get_plan()
        action_feedback = None
        self.end_event_action_service()
        self.action_feedback_subscriber()

        while True:
            rospy.loginfo('{}: Executing state EndActionEvent')
            if self.__shutdown__:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.rm.before_shutdown()
                return 'shutdown'
            
            if self.action_succeeded:
                self.end_event_action_srv.shutdown()
                self.af_sub.unregister()
                return 'ok'

            if self.task_succeeded:
                # killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                return 'ok'
            
            if self.task_failed:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                return 'shutdown'
            rospy.sleep(1)


class FirstAid(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, task_kb_id):
        TaskER.BlockingState.__init__(self, input_keys=['input_destination'], output_keys=['output_destination'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])
        self.task_kb_id = task_kb_id
        self.action_id = None
        self.plan_actions = None
        self.task_succeeded = False
        self.task_failed = False
        self.action_succeeded = False
        self.rm = rosplan_manager(task_id=self.task_kb_id, task_name="rescue")

    def rescue_action_service_handler(self, req):
        t = 0
        t_limit = 10
        while t < t_limit:
            rospy.sleep(1)
            print("First Aid progress: " + str(t) + "/" + str(t_limit) + ".") 
            t += 1
        return 1

    def rescue_action_service(self):
        self.rs_service = rospy.Service('/rescue_finished_service', RescueService, self.rescue_action_service_handler)

    def action_feedback_subscriber(self):
        aft = "/rosplan_plan_dispatcher/action_feedback"
        self.af_sub = rospy.Subscriber(aft, ActionFeedback, self.action_feedback_cb)

    def action_feedback_cb(self, action_feedback):
        (action_id, action_status) = self.get_feedback_action_params(action_feedback)
        # rospy.loginfo('{}: First aid in progress...')
        if action_status == 1:
            print("Action " + str(action_id) + " enabled. (status 1)")
        if action_status == 2 and int(action_id) != int(self.plan_length)-1:
            print("Action " + str(action_id) + " succeeded to goal state. (status 2) plan len != action id")
            self.action_succeeded = True
        if action_status == 10:
            print("Action " + str(action_id) + " failed. (status 10)")
            self.task_failed = True
                
    def transition_function(self, userdata):
        self.destination = userdata.input_destination
        userdata.output_destination = userdata.input_destination
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        self.plan_length = self.rm.get_plan()
        action_feedback = None
        self.action_feedback_subscriber()
        self.rescue_action_service()
        
        while True:
            print("Im offering the first aid in " + str(self.destination) + ".")
            if self.__shutdown__:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.rm.before_shutdown()
                return 'shutdown'

            if self.action_succeeded:
                self.af_sub.unregister()
                self.rs_service.shutdown()
                return 'ok'
            
            if self.task_failed:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                return 'shutdown'
            rospy.sleep(1)

    def get_feedback_action_params(self, action_feedback):
        action_id = action_feedback.action_id
        plan_id = action_feedback.plan_id
        action_status = action_feedback.status
        return (action_id, action_status)

class Rescue(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places, task_kb_id, destination):        

        smach_rcprg.StateMachine.__init__(self, input_keys=['susp_data'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        
        self.sis = smach_ros.IntrospectionServer(unicode(str("/rescue_smach_view_server")), self, unicode("rescue"))
        self.sis.start()

        with self:

            smach_rcprg.StateMachine.add('ExecuteROSPlan', ExecuteROSPlan(sim_mode, conversation_interface, task_kb_id, destination),
                                    transitions={'ok':'MoveToVictim', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'output_destination':'destination'})

            smach_rcprg.StateMachine.add('MoveToVictim', MoveToVictim(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok':'EndActionMoveToVictimEvent', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination'})

            smach_rcprg.StateMachine.add('EndActionMoveToVictimEvent', EndActionEvent(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok':'FirstAid', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination'})

            smach_rcprg.StateMachine.add('FirstAid', FirstAid(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok':'EndActionFirstAidEvent', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination'})

            smach_rcprg.StateMachine.add('EndActionFirstAidEvent', EndActionEvent(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination'})

            # self.sis = smach_ros.IntrospectionServer(unicode(str("/rescue_smach_view_server")), self, unicode("rescue"))
            # self.sis.start()