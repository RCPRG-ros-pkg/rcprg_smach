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
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose

import rcprg_smach.navigation
from TaskER.TaskER import TaskER
from rcprg_smach import smach_rcprg

from pl_nouns.dictionary_client import DisctionaryServiceClient

import rcprg_smach.task_manager

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

class SetHumanAndDestination(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        TaskER.BlockingState.__init__(self, input_keys=['human_name'], output_keys=['human_pose', 'dest_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Znajduję człowieka'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        # self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Ustalam gdzie jest człowiek' )
        userdata.human_pose = rcprg_smach.navigation.PoseDescription({'place_name':unicode(userdata.human_name)})

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class CheckHumanState(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        TaskER.BlockingState.__init__(self, input_keys=['human_name'], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Sprawdzam stan człowieka'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        gender = ""
        if userdata.human_name in ["John", "Peter"]:
            gender = "powinien Pan"
        else:
            gender = "powinna Pani"
        if isinstance(userdata.human_name, str):
            userdata.human_name = userdata.human_name.decode('utf-8')
        userdata.human_name = userdata.human_name.encode('utf-8').decode('utf-8')


        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe '+userdata.human_name+u', jak się czujesz?' )
        rospy.sleep(2)
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Dziękuję za informację. Dowidzenia.x' )
        if self.__shutdown__:
            return 'shutdown'
        return 'ok'


class SayIFinished(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        TaskER.BlockingState.__init__(self,
                             outcomes=['ok', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Mówię, że zakończyłem'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
  #      self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe zakończyłem zadanie' )

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class ExcuseHuman(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        TaskER.BlockingState.__init__(self, input_keys=['human_name', 'current_pose'],
                             outcomes=['ok', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Przepraszam człowieka. Mówię, że zaraz wracam'
        self.kb_places = kb_places
        self.sim_mode = sim_mode
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        if isinstance(userdata.human_name, str):
            human_name = userdata.human_name.decode('utf-8')
        human_name = userdata.human_name.encode('utf-8').decode('utf-8')

        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Przepraszam Cię '+human_name+u' , mam pilne zadanie. Jadę ratować czlowieka, zaraz wracam.' )
        if self.sim_mode in ['sim', 'gazebo']:
            mc = self.kb_places.getMapContext('sim')
        elif self.sim_mode == 'real':
            mc = self.kb_places.getMapContext('real')
        else:
            raise Exception('<suspend_gh> I dont know the map context you use: "' + self.sim_mode + '". I know <sim> and <gazebo> and <real>.')

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class SuspGH(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        rcprg_smach.smach_rcprg.StateMachine.__init__(self, input_keys=['fsm_es','susp_data'], output_keys=['susp_data'],
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
            rcprg_smach.smach_rcprg.StateMachine.add(u'SetHeightMid', rcprg_smach.navigation.SetHeight(sim_mode, conversation_interface),
                                    transitions={'ok':'SetHumanAndDestination', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'torso_height':'default_height'})

            rcprg_smach.smach_rcprg.StateMachine.add(u'SetHumanAndDestination', SetHumanAndDestination(sim_mode, conversation_interface),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'human_name':'fsm_es'})

            rcprg_smach.smach_rcprg.StateMachine.add(u'SetNavParams', rcprg_smach.navigation.SetNavParams(sim_mode),
                                    transitions={'ok':'RememberCurrentPose', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            rcprg_smach.smach_rcprg.StateMachine.add(u'RememberCurrentPose', rcprg_smach.navigation.RememberCurrentPose(sim_mode),
                                    transitions={'ok':'TurnAround', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'current_pose':'current_pose'})

            rcprg_smach.smach_rcprg.StateMachine.add(u'TurnAround', rcprg_smach.navigation.TurnAround(sim_mode, conversation_interface),
                                    transitions={'ok':'RememberHumanAwaitingPose', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown', 'stall':'FAILED'},
                                    remapping={'current_pose':'current_pose'})
            
            rcprg_smach.smach_rcprg.StateMachine.add(u'RememberHumanAwaitingPose', rcprg_smach.navigation.RememberCurrentPose(sim_mode),
                                    transitions={'ok':'ExcuseHuman', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'current_pose':'current_pose'})

            rcprg_smach.smach_rcprg.StateMachine.add(u'ExcuseHuman', ExcuseHuman(sim_mode, conversation_interface, kb_places),
                                    transitions={'ok':'SayIFinished','shutdown':'shutdown', },
                                    remapping={'human_name':'fsm_es','current_pose':'current_pose'})


            # rcprg_smach.smach_rcprg.StateMachine.add('SetHeightLow', rcprg_smach.navigation.SetHeight(sim_mode, conversation_interface),
            #                         transitions={'ok':'AskForGoods', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'torso_height':'lowest_height'})

            # rcprg_smach.smach_rcprg.StateMachine.add('AskForGoods', SayAskForGoods(sim_mode, conversation_interface),
            #                         transitions={'ok':'MoveToDestination', 'preemption':'PREEMPTED', 'error':'FAILED',
            #                         'timeout':'AskForGoods','shutdown':'shutdown', 'turn_around':'TurnAroundA1'},
            #                         remapping={'goods_name':'goal', 'q_load_answer_id':'q_load_answer_id'})

            # rcprg_smach.smach_rcprg.StateMachine.add('MoveBack', rcprg_smach.navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
            #                         transitions={'FINISHED':'SayTakeGoods', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'goal':'initial_pose', 'susp_data':'susp_data'})

            # rcprg_smach.smach_rcprg.StateMachine.add('SayTakeGoods', SayTakeGoods(sim_mode, conversation_interface),
            #                         transitions={'ok':'SetHeightEnd', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown', 'timeout':'SayTakeGoods', 'turn_around':'TurnAroundB1'},
            #                         remapping={'goods_name':'goal', 'q_load_answer_id':'q_load_answer_id'})

            # rcprg_smach.smach_rcprg.StateMachine.add('TurnAroundA1', rcprg_smach.navigation.RememberCurrentPose(sim_mode),
            #                         transitions={'ok':'TurnAroundA2', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'current_pose':'current_pose'})

            # rcprg_smach.smach_rcprg.StateMachine.add('TurnAroundA2', rcprg_smach.navigation.TurnAround(sim_mode, conversation_interface),
            #                         transitions={'ok':'AskForGoods', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown', 'stall':'AskForGoods'},
            #                         remapping={'current_pose':'current_pose'})

            # rcprg_smach.smach_rcprg.StateMachine.add('TurnAroundB1', rcprg_smach.navigation.RememberCurrentPose(sim_mode),
            #                         transitions={'ok':'TurnAroundB2', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'current_pose':'current_pose'})

            # rcprg_smach.smach_rcprg.StateMachine.add('TurnAroundB2', rcprg_smach.navigation.TurnAround(sim_mode, conversation_interface),
            #                         transitions={'ok':'SayTakeGoods', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown', 'stall':'AskForGoods'},
            #                         remapping={'current_pose':'current_pose'})

            # rcprg_smach.smach_rcprg.StateMachine.add('SetHeightEnd', rcprg_smach.navigation.SetHeight(sim_mode, conversation_interface),
            #                         transitions={'ok':'SayIFinished', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'torso_height':'default_height'})

            rcprg_smach.smach_rcprg.StateMachine.add('SayIFinished', SayIFinished(sim_mode, conversation_interface),
                                    transitions={'ok':'FINISHED', 'shutdown':'shutdown'})
    def my_exe(self):
        pass
    # def execute(self, userdata):
    #     while not rospy.is_shutdown():
    #         print "GOODS: ", userdata.susp_data.getData()
    #         if userdata.susp_data.getData() == "suspension requirements from the task harmoniser":
    #             return 'SUSPENDED'
    #         rospy.sleep(1)
    #     print "END GOODS"
        # self.description = u'Podaję {"' + userdata.goal + u'", biernik}'
        # return super(BringGoods, self).execute(userdata)
