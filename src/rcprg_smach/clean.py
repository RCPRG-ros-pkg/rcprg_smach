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
from std_srvs.srv import Empty
 
import os
import task_manager
import threading
import time
import subprocess
# from subprocess import DEVNULL, PIPE, STDOUT
import multiprocessing
import psutil
import shlex

class ExecuteROSPlan(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, task_kb_id, destination):
        TaskER.BlockingState.__init__(self, input_keys=[], output_keys=['output_destination'],
                            outcomes=['stpt', 'kuchnia', 'salon', 'pokoj', 'warsztat', 'preemption', 'error', 'shutdown'])
        self.task_kb_id = task_kb_id
        self.sim_mode = sim_mode
        self.conversation_interface = conversation_interface
        self.destination = destination
        self.plan_actions = None
        self.first_goal = None
        self.action_id = None
        self.rm = rosplan_manager(task_id=self.task_kb_id, task_name='clean')

        # print("Calling Actinf execution")
        # FNULL = open(os.devnull, 'w')
        # DEVNULL = open(os.devnull, 'wb')
        # # subp = subprocess.Popen(shlex.split("roslaunch tiago_rosplan_sim launch_actinf.launch"), shell=False)
        # subp = subprocess.Popen(shlex.split("roslaunch tiago_rosplan_sim launch_actinf.launch"), stdout=DEVNULL)
        # p = psutil.Process(subp.pid)
        # print("Waiting for Actinf to setup")
        # rospy.sleep(2)
    
    def action_feedback_cb(self, action_feedback):
        (action_id, action_status) = self.get_feedback_action_params(action_feedback)
        if action_status == 1 and self.plan_actions is not None:
            print("Action " + str(action_id) + " enabled. (status 1)")
            self.first_goal = self.plan_actions[0]
    
    def complete_plan_cb(self, complete_plan):
        if self.plan_actions is None:
            self.plan_actions = {}
            for action in complete_plan.plan:
                for param in action.parameters:
                    if param.key == "end-loc":
                        self.plan_actions[action.action_id] = param.value
                        break
    
    def get_feedback_action_params(self, action_feedback):
        action_id = action_feedback.action_id
        plan_id = action_feedback.plan_id
        action_status = action_feedback.status
        return (action_id, action_status)
 
    def action_feedback_subscriber(self):
        aft = "/rosplan_plan_dispatcher/action_feedback"
        self.af_sub = rospy.Subscriber(aft, ActionFeedback, self.action_feedback_cb)
    
    def complete_plan_subscriber(self):
        cpt = "/rosplan_parsing_interface/complete_plan"
        self.cp_sub = rospy.Subscriber(cpt, CompletePlan, self.complete_plan_cb)
     
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        rospy.sleep(10)
        rpl_ei = ROSPlanExecInterface(da_type="clean", da_id = self.task_kb_id, goal=self.destination)
        # if the task is being restarted
        # if self.rm.if_restart:
        # for i in range(10):
        #     print("task is being restarted")
        # rpl_ei.run_rosplan_if_restart()
        # else:
        #     for i in range(10):
        #         print("task is being run for the first time")
        rpl_ei.run_plan_parse_and_dispatch()
        
        # rospy.sleep(2)
        self.complete_plan_subscriber()
        self.action_feedback_subscriber()
        print("Inside Execute ROSPlan transition function")
        while True:
            if self.__shutdown__:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                self.rm.before_finished_shutdown()
                return 'shutdown'
            
            if self.first_goal is not None:
                userdata.output_destination = self.first_goal
                self.af_sub.unregister()
                self.cp_sub.unregister()
                return self.first_goal
 
class MoveTo(TaskER.SuspendableState):
    def __init__(self, sim_mode, conversation_interface, task_kb_id):
        TaskER.SuspendableState.__init__(self, input_keys=['input_destination', 'counter_in'], output_keys=['output_destination', 'counter_out'],
                            outcomes=['ok', 'preemption', 'error', 'shutdown'])
        self.task_kb_id = task_kb_id
        self.plan_actions = None
        self.action_succeeded = False
        self.task_failed = False
        self.action_id = None
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
            self.action_succeeded = True
        if action_status == 10:
            print("Action " + str(action_id) + " failed. (status 10)")
            self.task_failed = True
            
    def get_feedback_action_params(self, action_feedback):
        action_id = action_feedback.action_id
        plan_id = action_feedback.plan_id
        action_status = action_feedback.status
        return (action_id, action_status)
    
    def cancel_dispatch(self):
        rospy.wait_for_service('/rosplan_plan_dispatcher/cancel_dispatch')
        try:
            cancel_service_client = rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch', Empty)
            cancel_service_client()
            # for i in range(5):
            #     print("CALLED CANCEL DISPATCH SERVICE")
        except rospy.ServiceException as e:
            print("Service cancel call failed: %s"%e)
            # for i in range(5):
            #     print("CANCEL SERVICE CALL DISPATCH FAILED")
    
    def cancel_actinf(self):
        rospy.wait_for_service('/rosplan_action_interface/cancel_interface')
        try:
            # for i in range(5):
            #     print("CALLING CANCEL ACTINF SERVICE")
            cancel_service_client = rospy.ServiceProxy('/rosplan_action_interface/cancel_interface', Empty)
            cancel_service_client()
            # for i in range(5):
            #     print("CALLED CANCEL ACTINF SERVICE")
        except rospy.ServiceException as e:
            print("Service cancel call failed: %s"%e)
            # for i in range(5):
            #     print("CANCEL SERVICE CALL ACTINF FAILED")

        
    def transition_function(self, userdata):
        # if self.is_suspension_flag() != None:
        #     for i in range(20):
        #         print("IS SUSPENSION FLAG!")
        #     self.request_preempt()
        #     return 'preemption'

        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        self.destination = str(userdata.input_destination)
        userdata.output_destination = self.destination
        self.action_id = userdata.counter_in
        userdata.counter_out = userdata.counter_in + 1
        action_feedback = None
        self.action_feedback_subscriber()
        print("Inside MoveTo transition function")
        while True:
            print("Heading to the " + self.destination + " in progress...")
            if self.is_suspension_flag() != None and not self.__shutdown__:
                for i in range(5):
                    print("IS SUSPENSION FLAG")
                    print(self.is_suspension_flag())
            # if self.preempt_requested():
            #     for i in range(5):
            #         print("PREEMPT REQUESTED")
            #         print(self.preempt_requested())
                self.cancel_dispatch()
                rospy.sleep(1)
                self.cancel_actinf()
                rospy.sleep(2)
                self.rm.before_shutdown()
                rospy.sleep(2)
                # self.rm.kill_actinf()
                self.request_preempt()
                # self.service_preempt()
                return 'preemption'

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
        TaskER.BlockingState.__init__(self, input_keys=['input_destination', 'counter_in'], output_keys=['output_destination', 'counter_out'],
                             outcomes=['stpt', 'kuchnia', 'salon', 'pokoj', 'warsztat', 'ok', 'preemption', 'error', 'shutdown'])
        self.task_kb_id = task_kb_id
        self.action_id = None
        self.next_action = None
        self.plan_actions = None
        self.rm = rosplan_manager(task_id=self.task_kb_id, task_name='clean')

    def end_event_action_service_handler(self, req):
        # for i in range(10):
        #     print("ACTION ID IN END EVENT HANDLER ")
        #     print(self.action_id)
        # self.rm.update_goal(self.action_id-1)
        self.rm.remove_goal(self.action_id-1)
        return 1

    def end_event_action_service(self):
        self.end_event_action_srv = rospy.Service('/rescue_service', RescueService, self.end_event_action_service_handler)

    def action_feedback_subscriber(self):
        aft = "/rosplan_plan_dispatcher/action_feedback"
        self.af_sub = rospy.Subscriber(aft, ActionFeedback, self.action_feedback_cb)
 
    def action_feedback_cb(self, action_feedback):
        (action_id, action_status) = self.get_feedback_action_params(action_feedback)
        if self.plan_actions is not None:
            if action_status == 1:
                print("Action " + str(action_id) + " enabled. (status 1)")
            if action_status == 2 and int(action_id) != int(self.plan_length)-1:
                print("Action " + str(action_id) + " succeeded to goal state. (status 2) plan len != action id")
                self.action_succeeded = True
                self.next_action = self.plan_actions[action_id+1]
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
    
    def complete_plan_subscriber(self):
        cpt = "/rosplan_parsing_interface/complete_plan"
        self.cp_sub = rospy.Subscriber(cpt, CompletePlan, self.complete_plan_cb)
    
    def complete_plan_cb(self, complete_plan):
        if self.plan_actions is None:
            self.plan_actions = {}
            for action in complete_plan.plan:
                for param in action.parameters:
                    if param.key == "end-loc":
                        self.plan_actions[action.action_id] = param.value
                        break
    
    def cancel_dispatch(self):
        rospy.wait_for_service('/rosplan_plan_dispatcher/cancel_dispatch')
        try:
            cancel_service_client = rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch', Empty)
            cancel_service_client()
            for i in range(5):
                print("CALLED CANCEL DISPATCH SERVICE")
        except rospy.ServiceException as e:
            print("Service cancel call failed: %s"%e)
            for i in range(5):
                print("CANCEL SERVICE CALL DISPATCH FAILED")
    
    def cancel_actinf(self):
        rospy.wait_for_service('/rosplan_action_interface/cancel_interface')
        try:
            cancel_service_client = rospy.ServiceProxy('/rosplan_action_interface/cancel_interface', Empty)
            cancel_service_client()
            for i in range(5):
                print("CALLED CANCEL ACTINF SERVICE")
        except rospy.ServiceException as e:
            print("Service cancel call failed: %s"%e)
            for i in range(5):
                print("CANCEL SERVICE CALL ACTINF FAILED")

                
    def transition_function(self, userdata):
        # if self.is_suspension_flag() != None:
        #     for i in range(20):
        #         print("IS SUSPENSION FLAG!")
        #     self.request_preempt()
        #     return 'preemption'
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        self.description = u'Executing state EndActionEvent'
        self.destination = str(userdata.input_destination)
        self.action_id = userdata.counter_in
        userdata.counter_out = userdata.counter_in + 1
        self.rm = rosplan_manager(task_id=self.task_kb_id, task_name="clean")
        self.plan_length = self.rm.get_plan()
        action_feedback = None
        self.task_succeeded = False
        self.task_failed = False
        self.action_succeeded = False
        self.end_event_action_service()
        self.complete_plan_subscriber()
        self.action_feedback_subscriber()
        print("Inside EndActionEvent transition function")
 
        while True:
            rospy.loginfo('{}: Executing state EndActionEvent')
            # if self.is_suspension_flag() != None and not self.__shutdown__ and not self.task_succeeded:
            #     self.cancel_dispatch()
            #     rospy.sleep(1)
            #     self.cancel_actinf()
            #     rospy.sleep(2)
            #     self.rm.before_shutdown()
            #     rospy.sleep(2)
            #     # self.rm.kill_actinf()
            #     self.request_preempt()
            #     return 'preemption'
            if self.preempt_requested():
                self.service_preempt()
                return 'preemption'

            if self.__shutdown__:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: clean SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                return 'shutdown'
            
            if self.action_succeeded:
                self.af_sub.unregister()
                self.cp_sub.unregister()
                self.end_event_action_srv.shutdown()
                userdata.output_destination = self.next_action
                return self.next_action
 
            if self.task_succeeded:
                # killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: clean SMACH shutdown request.')
                self.af_sub.unregister()
                self.cp_sub.unregister()
                self.end_event_action_srv.shutdown()
                self.rm.before_finished_shutdown()
                # self.rm.kill_actinf()
                # self.cancel_dispatch()
                return 'ok'
            
            if self.task_failed:
                # saving KB state, killing ROSPlan and action interface nodes and sending vel=0 to mobile base
                rospy.loginfo('{}: clean SMACH shutdown request.')
                self.rm.before_finished_shutdown()
                # self.rm.kill_actinf()
                # self.cancel_dispatch()
                return 'shutdown'
            rospy.sleep(1)
    
 
class Clean(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places, task_kb_id, destination):        
 
        smach_rcprg.StateMachine.__init__(self, input_keys=['susp_data'], output_keys=['susp_data'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        self.userdata.action_counter = 0 
        with self:
            smach_rcprg.StateMachine.add('ExecuteROSPlan', ExecuteROSPlan(sim_mode, conversation_interface, task_kb_id, destination),
                                    transitions={'stpt':'MoveToStpt', 'kuchnia':'MoveToKuchnia', 'salon':'MoveToSalon', 'pokoj':'MoveToPokoj', 
                                    'warsztat':'MoveToWarsztat', 'preemption':'PREEMPTED', 'error': 'FAILED','shutdown':'shutdown'},
                                    remapping={'output_destination':'destination'})
 
            smach_rcprg.StateMachine.add('MoveToKuchnia', MoveTo(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok': 'EndActionMoveEvent', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination',
                                    'counter_in':'action_counter', 'counter_out':'action_counter'})
            
            smach_rcprg.StateMachine.add('MoveToSalon', MoveTo(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok': 'EndActionMoveEvent', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination',
                                    'counter_in':'action_counter', 'counter_out':'action_counter'})
 
            smach_rcprg.StateMachine.add('MoveToPokoj', MoveTo(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok': 'EndActionMoveEvent', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination',
                                    'counter_in':'action_counter', 'counter_out':'action_counter'})
 
            smach_rcprg.StateMachine.add('MoveToWarsztat', MoveTo(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok': 'EndActionMoveEvent', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination',
                                    'counter_in':'action_counter', 'counter_out':'action_counter'})
            
            smach_rcprg.StateMachine.add('MoveToStpt', MoveTo(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'ok': 'EndActionMoveEvent', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination',
                                    'counter_in':'action_counter', 'counter_out':'action_counter'})
 
            smach_rcprg.StateMachine.add('EndActionMoveEvent', EndActionEvent(sim_mode, conversation_interface, task_kb_id),
                                    transitions={'stpt':'MoveToStpt', 'kuchnia':'MoveToKuchnia', 'salon':'MoveToSalon', 'pokoj':'MoveToPokoj', 
                                    'warsztat':'MoveToWarsztat', 'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'input_destination':'destination', 'output_destination':'destination',
                                    'counter_in':'action_counter', 'counter_out':'action_counter'})
                                    
            # self.sis = smach_ros.IntrospectionServer(unicode(str("/clean_smach_view_server")), self, unicode("clean"))
            # self.sis.start()
