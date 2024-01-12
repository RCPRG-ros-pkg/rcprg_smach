#!/usr/bin/env python3
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

import navigation
import human_fell
from TaskER.TaskER import TaskER
from rcprg_smach import smach_rcprg

from pl_nouns.dictionary_client import DisctionaryServiceClient

import task_manager

ACK_WAIT_MAX_TIME_S = 30

def makePose(x, y, theta):
    q = quaternion_from_euler(0, 0, theta)
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.orientation.x = q[0]
    result.orientation.y = q[1]
    result.orientation.z = q[2]
    result.orientation.w = q[3]
    return result
def ApproachHuman()

class ApproachHuman(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        self.conversation_interface = conversation_interface

        TaskER.SuspendableState.__init__(self,
                             outcomes=['ok', 'shutdown'],
                             input_keys=['susp_data'])

        self.description = u'Podjeżdżam do człowieka'

    def transition_function(self, userdata):
        approach_service = rospy.ServiceProxy('approach_person',)

        client_autonomous_head = actionlib.SimpleActionClient('/pal_head_manager/disable', DisableAction)
        client_autonomous_head.wait_for_server()
        client_autonomous_head.send_goal(DisableGoal())

        # move head to detect human on the floor
        client_move_head = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
        client_move_head.wait_for_server()
        point_head_goal = PointHeadGoal()
        point_head_goal.target.header.frame_id = 'base_link'
        point_head_goal.target.point.x = 1.5
        point_head_goal.pointing_axis.z = 1
        point_head_goal.pointing_frame = 'xtion_rgb_optical_frame'
        point_head_goal.min_duration.secs = 1
        point_head_goal.max_velocity = 1
        client_move_head.send_goal(point_head_goal)

        # run safe approach to human action
        approach_service()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class HumanFell(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.StateMachine.__init__(self, input_keys=['human_name','susp_data'], output_keys=['susp_data'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5
        # TODO: use knowledge base for this:
        self.userdata.default_height = 0.2
        self.userdata.lowest_height = 0.0

        self.description = u'Podaję rzecz'

        with self:

            smach_rcprg.StateMachine.add('SetHumanAndDestination', human_fell.SetHumanAndDestination(sim_mode, conversation_interface),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(sim_mode),
                                    transitions={'ok':'MoveToHuman', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach_rcprg.StateMachine.add('MoveToHuman', navigation.MoveToHumanComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'ApproachHuman', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'human_pose', 'susp_data':'susp_data'})

            smach_rcprg.StateMachine.add('ApproachHuman', ApproachHuman(sim_mode, conversation_interface),
                                    transitions={'ok':'CheckHumanState','shutdown':'shutdown'},
                                    remapping={'susp_data':'susp_data'})

            smach_rcprg.StateMachine.add('CheckHumanState', human_fell.CheckHumanState(sim_mode, conversation_interface),
                                    transitions={'ok':'SayIFinished', 'preemption':'PREEMPTED', 'error':'FAILED'
                                    ,'shutdown':'shutdown', },
                                    remapping={'human_name':'human_name'})

            smach_rcprg.StateMachine.add('SayIFinished', human_fell.SayIFinished(sim_mode, conversation_interface),
                                    transitions={'ok':'FINISHED', 'shutdown':'shutdown'})
