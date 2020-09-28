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
import tiago_torso_controller
import smach_rcprg

from task_manager import PoseDescription

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

class PickPose(smach_rcprg.State):
    def __init__(self, sim_mode, kb_places):
        smach_rcprg.State.__init__(self, input_keys=['in_current_pose'], output_keys=['nav_goal_task'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        assert sim_mode in ['sim', 'gazebo', 'real']
        self.sim_mode = sim_mode
        self.kb_places = kb_places

        self.description = u'Wybieram punkt docelowy'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        places = [u'kuchnia', u'warsztat', u'pokój', u'salon']
        place_name = random.choice( places )

        userdata.nav_goal_task = PoseDescription({'place_name':place_name})

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class Wander(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.StateMachine.__init__(self, outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5
        self.userdata.default_height = 0.2

        self.description = u'Patroluję'

        with self:
            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(sim_mode),
                                        transitions={'ok':'SetHeight', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                        'shutdown':'shutdown'},
                                        remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach_rcprg.StateMachine.add('SetHeight', tiago_torso_controller.SetHeight(sim_mode, conversation_interface),
                                        transitions={'ok':'PickPose', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                        'shutdown':'shutdown'},
                                        remapping={'torso_height':'default_height'})

            smach_rcprg.StateMachine.add('PickPose', PickPose(sim_mode, kb_places),
                                        transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                        'shutdown':'shutdown'},
                                        remapping={'nav_goal_task':'nav_goal_task'})

            smach_rcprg.StateMachine.add('MoveTo', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                        transitions={'FINISHED':'PickPose', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                        'shutdown':'shutdown'},
                                        remapping={'goal':'nav_goal_task'})
