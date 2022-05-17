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

import navigation
from TaskER.TaskER import TaskER
from rcprg_smach import smach_rcprg

from pl_nouns.dictionary_client import DisctionaryServiceClient

import task_manager

class BringGoodsNew(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places, place_name):
        smach_rcprg.StateMachine.__init__(self, input_keys=['goal','susp_data'], output_keys=['susp_data'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])


        self.userdata.max_lin_vel = 0.3
        self.userdata.max_lin_accel = 0.8
        print(place_name)

        self.userdata.place_pose = navigation.PoseDescription({'place_name': place_name})


        with self:
            smach_rcprg.StateMachine.add('RememberCurrentPose', navigation.RememberCurrentPose(sim_mode),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'current_pose':'initial_pose'})

            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(sim_mode),
                                    transitions={'ok':'MoveToPlace', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach_rcprg.StateMachine.add('MoveToPlace', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'MoveBack', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'place_pose'})

            smach_rcprg.StateMachine.add('MoveBack', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'FINISHED', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'initial_pose'})





