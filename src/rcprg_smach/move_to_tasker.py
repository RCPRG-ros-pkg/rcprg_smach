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
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
import tiago_msgs.msg
import std_msgs

from task_database.srv import GetParamsForScenario, AddParamsForScenario, CloneScenarioWithNewIntent
from language_processor.srv import InitiateConvBasedOnCtx
from rico_context.srv import GetContext, ResetContext, ResetContextResponse, ResetContextRequest
from rico_context.msg import HistoryEvent

import navigation
from TaskER.TaskER import TaskER
from rcprg_smach import smach_rcprg
from pl_nouns.dictionary_client import DisctionaryServiceClient
import task_manager
import os
from task_database.srv import GetTaskDescription

ACK_WAIT_MAX_TIME_S = 30

pub_context = rospy.Publisher('/context/push', HistoryEvent, queue_size=10)


class SayIFinished(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        TaskER.BlockingState.__init__(self, input_keys=[], output_keys=[],
                                      outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.reset_context = rospy.ServiceProxy('/context/reset', ResetContext)

        self.description = u'Mówię, że zakończyłem'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(
            rospy.get_name(), self.__class__.__name__))

        self.conversation_interface.speakNowBlocking(
            u'I finished performing the task')

        self.reset_context()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'


class MoveToPlace(smach_rcprg.StateMachine):

    def __init__(self, sim_mode, conversation_interface, kb_places, task_parameters):
        input_keys = []

        for idx in range(0, len(task_parameters), 2):
            param_name = task_parameters[idx]
            param_value = task_parameters[idx+1]
            
            input_keys.append(param_name)

        input_keys.extend(['susp_data', 'goal']);

        smach_rcprg.StateMachine.__init__(self, input_keys=input_keys, output_keys=['susp_data'],
                                          outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])

        for idx in range(0, len(task_parameters), 2):
            param_name = task_parameters[idx]
            param_value = task_parameters[idx+1]

            print('Parameters setting to userdate: ', param_name, param_value)

            setattr(self.userdata, param_name, param_value)

        self.userdata.max_lin_vel = 0.4
        self.userdata.max_lin_accel = 0.5

        self.userdata.default_height = 0.2
        self.userdata.lowest_height = 0.0

        self.userdata.goal = task_manager.PoseDescription({'place_name': self.userdata.place})

        self.description = u'Going to a place'

        with self:
            smach_rcprg.StateMachine.add('MoveToPlace', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                         transitions={'FINISHED': 'SayIFinished', 'PREEMPTED': 'PREEMPTED', 'FAILED': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={'susp_data': 'susp_data'})

            smach_rcprg.StateMachine.add('SayIFinished', SayIFinished(sim_mode, conversation_interface),
                                         transitions={'ok': 'FINISHED', 'shutdown': 'shutdown', 'preemption': 'PREEMPTED', 'error': 'FAILED'})
