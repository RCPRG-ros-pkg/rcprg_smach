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
from TaskER.TaskER import TaskER
from rcprg_smach import smach_rcprg

from ROSPlan_communication.ROSPlanExecInterface import ROSPlanExecInterface
from ROSPlan_communication.rosplan_manager import rosplan_manager
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback, CompletePlan
from tiago_rosplan_sim.srv import RescueService

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
        self.sim_mode = sim_mode
        self.conversation_interface = conversation_interface
        self.destination = destination
        self.rm = rosplan_manager(task_id=self.task_kb_id, task_name="rescue")

        # print("Calling Actinf execution")
        # subp = subprocess.Popen(shlex.split("roslaunch tiago_rosplan_sim launch_actinf.launch"))
        # p = psutil.Process(subp.pid)
        # print("Waiting for Actinf to setup")
        # rospy.sleep(2)


    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        rospy.loginfo('ROSPlan running.')
        userdata.output_destination = self.destination
        self.description = u'Executing ROSPlan'
        rospy.loginfo('Executing ROSPlan')
        rpl_ei = ROSPlanExecInterface(da_type="move_new", da_id = self.task_kb_id, goal=self.destination)
        rpl_ei.run_plan_parse_and_dispatch()
        # rpl_ei.run_rosplan_if_restart()
        if self.__shutdown__:
            # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
            self.rm.before_finished_shutdown()
            return 'shutdown'
        return 'ok'


class MoveToDestination(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, task_kb_id):
        TaskER.BlockingState.__init__(self, input_keys=['input_destination'], output_keys=['output_destination'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])
        self.task_kb_id = task_kb_id
        self.action_succeeded = False
        self.task_failed = False
        self.action_id = None
        self.action_status = None
        self.rm = rosplan_manager(task_id=self.task_kb_id, task_name="rescue")

    def action_feedback_subscriber(self):
        aft = "/rosplan_plan_dispatcher/action_feedback"
        self.af_sub = rospy.Subscriber(aft, ActionFeedback, self.action_feedback_cb)

    def action_feedback_cb(self, action_feedback):
        (action_id, action_status) = self.get_feedback_action_params(action_feedback)
        rospy.loginfo('{}: Heading to destination in progress...')
        if action_status == 1:
            print("Action " + str(action_id) + " enabled. (status 1)")
        if action_status == 2:
            print("Action " + str(action_id) + " succeeded to goal state. (status 2) plan len != action id")
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
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        self.destination = str(userdata.input_destination)
        userdata.output_destination = userdata.input_destination
        action_feedback = None
        self.action_feedback_subscriber()
        while True:
            print("Heading to the " + self.destination + " in progress...")
            if self.__shutdown__:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: clean SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                # self.rm.kill_actinf()
                # self.cancel_dispatch()
                return 'shutdown'
            
            if self.action_succeeded:
                self.af_sub.unregister()
                return 'ok'
            
            if self.task_failed:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: clean SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                # self.rm.kill_actinf()
                # self.cancel_dispatch()
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
        # self.rm.remove_goal()
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
        
        if action_status == 2:
            print("Action " + str(action_id) + " succeeded to goal state. (status 2)")
            self.task_succeeded = True

        if action_status == 10:
            print("Action " + str(action_id) + " failed. (status 10)")
            self.task_failed = True
    
    def get_feedback_action_params(self, action_feedback):
        action_id = action_feedback.action_id
        plan_id = action_feedback.plan_id
        action_status = action_feedback.status
        return (action_id, action_status)
                    
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        self.description = u'Executing state EndActionEvent'
        self.destination = str(userdata.input_destination)
        userdata.output_destination = userdata.input_destination
        self.plan_length = self.rm.get_plan()
        action_feedback = None
        self.end_event_action_service()
        self.action_feedback_subscriber()

        while True:
            rospy.loginfo('{}: Executing state EndActionEvent')
            if self.__shutdown__:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                # self.rm.kill_actinf()
                # self.cancel_dispatch()
                return 'shutdown'
            
            if self.action_succeeded:
                self.af_sub.unregister()
                self.end_event_action_srv.shutdown()
                return 'ok'

            if self.task_succeeded:
                # killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.af_sub.unregister()
                self.end_event_action_srv.shutdown()
                self.rm.before_finished_shutdown()
                # self.rm.kill_actinf()
                # self.cancel_dispatch()
                return 'ok'
            
            if self.task_failed:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: rescue SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                # self.rm.kill_actinf()
                # self.cancel_dispatch()
                return 'shutdown'
            rospy.sleep(1)



class Movement(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places, task_kb_id, destination):        

        smach_rcprg.StateMachine.__init__(self, input_keys=['goal', 'susp_data'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        with self:

            smach_rcprg.StateMachine.add('ExecuteROSPlan', ExecuteROSPlan(sim_mode, conversation_interface, task_kb_id, destination),
                                    transitions={'ok':'MoveToDestination', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'output_destination':'destination'})

            smach_rcprg.StateMachine.add('MoveToDestination', MoveToDestination(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok':'EndActionEvent', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination'})

            smach_rcprg.StateMachine.add('EndActionEvent', EndActionEvent(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination'})
            # self.sis = smach_ros.IntrospectionServer(unicode(str("/move_smach_view_server")), self, unicode("move"))
            # self.sis.start()


# rostopic pub /rico_cmd tiago_msgs/Command "query_text: ''
# intent_name: 'MN'
# param_names: ['miejsce']
# param_values: ['pokoj']
# confidence: 0.0
# response_text: ''"