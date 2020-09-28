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
import smach_rcprg

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

class SetHumanAndDestination(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.State.__init__(self, input_keys=['human_name'], output_keys=['human_pose', 'dest_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Znajduję człowieka'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Ustalam gdzie jest człowiek' )
        if isinstance(userdata.human_name, str):
            human_name = userdata.human_name.decode('utf-8')
        human_name = userdata.human_name.encode('utf-8').decode('utf-8')
        userdata.human_pose = navigation.PoseDescription({'place_name':unicode(human_name)})

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class CheckHumanState(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.State.__init__(self, input_keys=['human_name'], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Sprawdzam stan człowieka'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        gender = ""
        if isinstance(userdata.human_name, str):
            human_name = userdata.human_name.decode('utf-8')
        human_name = userdata.human_name.encode('utf-8').decode('utf-8')
        if human_name in ["John", "Peter"]:
            gender = "powinien Pan"
        else:
            gender = "powinna Pani"


        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe '+human_name+u', jak się czujesz?' )
        rospy.sleep(2)
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Dziękuję za informację. Dowidzenia.x' )
        if self.__shutdown__:
            return 'shutdown'
        return 'ok'


class SayIFinished(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.State.__init__(self,
                             outcomes=['ok', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Mówię, że zakończyłem'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
      #  self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe zakończyłem zadanie' )

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

            smach_rcprg.StateMachine.add('SetHumanAndDestination', SetHumanAndDestination(sim_mode, conversation_interface),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(sim_mode),
                                    transitions={'ok':'MoveToHuman', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach_rcprg.StateMachine.add('MoveToHuman', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'CheckHumanState', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'human_pose', 'susp_data':'susp_data'})

            smach_rcprg.StateMachine.add('CheckHumanState', CheckHumanState(sim_mode, conversation_interface),
                                    transitions={'ok':'SayIFinished', 'preemption':'PREEMPTED', 'error':'FAILED'
                                    ,'shutdown':'shutdown', },
                                    remapping={'human_name':'human_name'})

            smach_rcprg.StateMachine.add('SayIFinished', SayIFinished(sim_mode, conversation_interface),
                                    transitions={'ok':'FINISHED', 'shutdown':'shutdown'})
