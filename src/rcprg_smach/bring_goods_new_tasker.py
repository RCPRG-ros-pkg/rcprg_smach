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


class SayAskKeeperForGoods(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, input_keys):
        TaskER.BlockingState.__init__(self, input_keys=input_keys, output_keys=['q_load_answer_id'],
                                      outcomes=['ok', 'preemption', 'error', 'shutdown', 'timeout', 'unexpected_question'])

        self.conversation_interface = conversation_interface

        self.description = u'Proszę o podanie rzeczy'
        self.get_params_for_scenario = rospy.ServiceProxy('get_params_for_scenario', GetParamsForScenario)

        self.asked = False

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(
            rospy.get_name(), self.__class__.__name__))
        
        if not self.asked:
            pub_context.publish(HistoryEvent('Rico', 'come to', 'keeper', ''))
            pub_context.publish(HistoryEvent('Keeper', 'see', 'Rico', ''))
            pub_context.publish(HistoryEvent('Keeper', 'say', 'Hi Rico, what you need?', ''))

        rospy.sleep(1.0)

        initiate_conv_based_on_ctx = rospy.ServiceProxy('initiate_conv_based_on_ctx', InitiateConvBasedOnCtx)
        add_params_for_scenario = rospy.ServiceProxy('add_params_for_scenario', AddParamsForScenario)
        clone_scenario_with_new_intent = rospy.ServiceProxy('clone_scenario_with_new_intent', CloneScenarioWithNewIntent)

        sentence = initiate_conv_based_on_ctx()

        # self.conversation_interface.speakNowBlocking(u'niekorzystne warunki pogodowe ' + sentence.sentence)
        self.conversation_interface.speakNowBlocking(sentence.sentence)

        print 'asked.'

        self.asked = True

        self.conversation_interface.addExpected('confirm')
        self.conversation_interface.addExpected('User gave')
        self.conversation_interface.addExpected('turn around')
        self.conversation_interface.addExpected('unexpected_question')

        # answer_id = self.conversation_interface.setAutomaticAnswer(
        #     'what are you doing', u'niekorzystne warunki pogodowe czekam na położenie przedmiotu')

        answer_id = self.conversation_interface.setAutomaticAnswer(
            'what are you doing', u'I\'m waiting for the item to be placed')

        userdata.q_load_answer_id = None

        start_time = rospy.Time.now()
        while True:
            end_time = rospy.Time.now()
            loop_time = end_time - start_time
            loop_time_s = loop_time.secs

            if self.__shutdown__:
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                return 'shutdown'

            if loop_time_s > ACK_WAIT_MAX_TIME_S:
                self.conversation_interface.removeExpected('confirm')
                self.conversation_interface.removeExpected('User gave')
                self.conversation_interface.removeExpected('turn around')
                self.conversation_interface.removeExpected('unexpected_question')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                return 'timeout'

            if self.preempt_requested():
                self.conversation_interface.removeExpected('confirm')
                self.conversation_interface.removeExpected('User gave')
                self.conversation_interface.removeExpected('turn around')
                self.conversation_interface.removeExpected('unexpected_question')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                self.service_preempt()
                return 'preemption'

            if self.conversation_interface.consumeExpected('confirm') or\
                    self.conversation_interface.consumeExpected('User gave'):
                self.conversation_interface.removeExpected('confirm')
                self.conversation_interface.removeExpected('User gave')
                self.conversation_interface.removeExpected('turn around')
                self.conversation_interface.removeExpected('unexpected_question')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                # self.conversation_interface.speakNowBlocking(
                #     u'niekorzystne warunki pogodowe thank you. now I need to transport goods to person who requested it. I need to go to place from which I started')
                # answer_id = self.conversation_interface.setAutomaticAnswer('what are you carrying', u'niekorzystne warunki pogodowe wiozę przedmiot')
                self.conversation_interface.speakNowBlocking(
                    u'thank you. now I need to transport goods to person who requested it. I need to go to place from which I started')
                answer_id = self.conversation_interface.setAutomaticAnswer('what are you carrying', u'I\'m carrying an item')
                userdata.q_load_answer_id = answer_id
                return 'ok'

            unexpected_question = self.conversation_interface.consumeExpected('unexpected_question')

            if unexpected_question:
                new_param = unexpected_question.param_values[0]

                is_specific_bringing = userdata.intent_name != 'bring item'

                if is_specific_bringing:
                    add_params_for_scenario(int(userdata.scenario_id), [new_param])
                else:
                    print 'userdata.item', userdata.item
                    item = userdata.item
                    new_intent_name = 'bring' + ' ' + item

                    print 'new_intent_name', new_intent_name

                    clone_scenario_with_new_intent(
                        int(userdata.scenario_id), new_intent_name, [new_param])
                    
                    print 'cloned'

                self.conversation_interface.removeExpected('unexpected_question')
                self.conversation_interface.removeExpected('confirm')
                self.conversation_interface.removeExpected('User gave')
                self.conversation_interface.removeExpected('turn around')

                self.conversation_interface.removeAutomaticAnswer(answer_id)
                # answer_id = self.conversation_interface.setAutomaticAnswer(
                #     'what are you doing', u'niekorzystne warunki pogodowe I\'m going to request additional information from user regarding this')
                answer_id = self.conversation_interface.setAutomaticAnswer(
                    'what are you doing', u'I\'m going to request additional information from user regarding this')

                # self.conversation_interface.speakNowBlocking(
                #     u'niekorzystne warunki pogodowe I\'m going to request additional information from user regarding this')
                self.conversation_interface.speakNowBlocking(
                    u'I\'m going to request additional information from user regarding this')

                userdata.q_load_answer_id = None

                return 'unexpected_question'

            if self.conversation_interface.consumeExpected('turn around'):
                self.conversation_interface.removeExpected('confirm')
                self.conversation_interface.removeExpected('User gave')
                self.conversation_interface.removeExpected('turn around')
                self.conversation_interface.removeExpected('unexpected_question')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                return 'turn_around'

            rospy.sleep(0.1)

        raise Exception('Unreachable code')


class TellInfoFromKeeper(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, input_keys):
        TaskER.BlockingState.__init__(self, input_keys=['q_load_answer_id']+input_keys, output_keys=[],
                                      outcomes=['ok', 'preemption', 'error', 'shutdown', 'timeout', 'turn_around', 'follow_up_answer', 'restart'])

        self.conversation_interface = conversation_interface
        self.description = u'Proszę o odebranie rzeczy'
        self.told = False

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(
            rospy.get_name(), self.__class__.__name__))

        # assert isinstance(userdata.item, unicode)

        # item = userdata.item

        # unexpected_question = self.conversation_interface.getContext(
        #     'unexpected_question')

        #self.conversation_interface.addSpeakSentence( u'Odbierz {"' + item + u'", biernik} i potwierdź' )

        if not self.told:
            pub_context.publish(HistoryEvent('Rico', 'came back to', 'senior', ''))
            pub_context.publish(HistoryEvent('Senior', 'see', 'Rico', ''))
            pub_context.publish(HistoryEvent('Senior', 'say', 'What\'s up with my request?', ''))

        rospy.sleep(1.0)

        initiate_conv_based_on_ctx = rospy.ServiceProxy('initiate_conv_based_on_ctx', InitiateConvBasedOnCtx)

        sentence = initiate_conv_based_on_ctx()

        # self.conversation_interface.speakNowBlocking(
        #     u'niekorzystne warunki pogodowe ' + sentence.sentence)

        self.conversation_interface.speakNowBlocking(sentence.sentence)

        print 'told info from keeper'

        self.told = True

        # if unexpected_question:
        #     self.conversation_interface.speakNowBlocking(
        #         u'niekorzystne warunki pogodowe Opiekun zadał pytanie.')
        #     self.conversation_interface.trigger_dialogflow_with_text(
        #         userdata.original_query)
        #     return 'restart'

        # else:
        #     self.conversation_interface.speakNowBlocking(
        #         u'niekorzystne warunki pogodowe odbierz {"' + item + u'", biernik} i potwierdź')

        self.conversation_interface.addExpected('confirm')
        self.conversation_interface.addExpected('User recieved')
        self.conversation_interface.addExpected('turn around')
        # self.conversation_interface.addExpected('follow_up_answer')

        # answer_id = self.conversation_interface.setAutomaticAnswer(
        #     'what are you doing', u'niekorzystne warunki pogodowe czekam na odebranie przedmiotu')

        answer_id = self.conversation_interface.setAutomaticAnswer(
            'what are you doing', u'I\'m waiting for the item to be taken')

        start_time = rospy.Time.now()
        while True:
            end_time = rospy.Time.now()
            loop_time = end_time - start_time
            loop_time_s = loop_time.secs

            if self.__shutdown__:
                return 'shutdown'

            if loop_time_s > ACK_WAIT_MAX_TIME_S:
                self.conversation_interface.removeExpected('confirm')
                self.conversation_interface.removeExpected('User recieved')
                self.conversation_interface.removeExpected('turn around')
                # self.conversation_interface.removeExpected('follow_up_answer')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                # Do not remove q_load_answer, because we want to enter this state again
                return 'timeout'

            if self.preempt_requested():
                self.conversation_interface.removeExpected('confirm')
                self.conversation_interface.removeExpected('User recieved')
                self.conversation_interface.removeExpected('turn around')
                # self.conversation_interface.removeExpected('follow_up_answer')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                if not userdata.q_load_answer_id is None:
                    self.conversation_interface.removeAutomaticAnswer(
                        userdata.q_load_answer_id)
                self.service_preempt()
                return 'preemption'

            if self.conversation_interface.consumeExpected('confirm') or\
                    self.conversation_interface.consumeExpected('User recieved'):
                self.conversation_interface.removeExpected('confirm')
                self.conversation_interface.removeExpected('User recieved')
                self.conversation_interface.removeExpected('turn around')
                # self.conversation_interface.removeExpected('follow_up_answer')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                if not userdata.q_load_answer_id is None:
                    self.conversation_interface.removeAutomaticAnswer(
                        userdata.q_load_answer_id)
                return 'ok'

            if self.conversation_interface.consumeExpected('turn around'):
                self.conversation_interface.removeExpected('confirm')
                self.conversation_interface.removeExpected('User recieved')
                self.conversation_interface.removeExpected('turn around')
                # self.conversation_interface.removeExpected('follow_up_answer')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                return 'turn_around'

            # if self.conversation_interface.consumeExpected('follow_up_answer'):
            #     [question_text] = unexpected_question.param_values
            #     print 'question text', question_text

            #     # TODO modyfikaja agenta

            #     # respTest = detect_intent_text('robot-rico-qrct', 'me', question_text, 'pl')
            #     # print respTest.query_result

            #     self.conversation_interface.removeExpected('ack')
            #     self.conversation_interface.removeExpected('ack_i_took')
            #     self.conversation_interface.removeExpected('turn_around')
            #     self.conversation_interface.removeExpected('follow_up_answer')
            #     self.conversation_interface.removeAutomaticAnswer(answer_id)
            #     if not userdata.q_load_answer_id is None:
            #         self.conversation_interface.removeAutomaticAnswer(
            #             userdata.q_load_answer_id)
            #     return 'follow_up_answer'

            rospy.sleep(0.1)

        raise Exception('Unreachable code')
    

class KillTask(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        TaskER.BlockingState.__init__(self, input_keys=[], output_keys=[],
                                      outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.reset_scenario_context = rospy.ServiceProxy('/context/reset_scenario', ResetContext)
        self.rico_process_last_intent_from_history = rospy.Publisher('/rico_process_last_intent_from_history', std_msgs.msg.Empty, queue_size=1)

        self.description = u'Zabijam zadanie'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(
            rospy.get_name(), self.__class__.__name__))

        self.reset_scenario_context()
        self.rico_process_last_intent_from_history.publish()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'


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
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        # self.conversation_interface.speakNowBlocking(
        #     u'niekorzystne warunki pogodowe I finished performing the task')

        self.conversation_interface.speakNowBlocking(
            u'I finished performing the task')

        # pub_context.publish(HistoryEvent('Rico', 'finish performing', '"bring goods" task', ''))
        # pub_context.publish(HistoryEvent('system', 'finish scenario', userdata.scenario_id, ''))

        self.reset_context()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'


class BringGoods(smach_rcprg.StateMachine):

    def __init__(self, sim_mode, conversation_interface, kb_places, task_parameters):
        rospy.wait_for_service('get_params_for_scenario')
        rospy.wait_for_service('initiate_conv_based_on_ctx')
        input_keys = []

        print 'task_parameters', task_parameters

        for idx in range(0, len(task_parameters), 2):
            param_name = task_parameters[idx]
            param_value = task_parameters[idx+1]
            
            input_keys.append(param_name)


        print 'Input keys passed to BringGoods: ', input_keys

        input_keys.extend(['susp_data', 'goal']);

        smach_rcprg.StateMachine.__init__(self, input_keys=input_keys, output_keys=['susp_data'],
                                          outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        

        for idx in range(0, len(task_parameters), 2):
            param_name = task_parameters[idx]
            param_value = task_parameters[idx+1]
            
            setattr(self.userdata, param_name, param_value)
        
        print 'Item from userdata: ', self.userdata.item

        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5
        # TODO: use knowledge base for this:

        self.userdata.kitchen_pose = navigation.PoseDescription({'place_name': u'kuchnia'})
        self.userdata.default_height = 0.2
        self.userdata.lowest_height = 0.0

        self.userdata.object_name = 'keeper'

        self.description = u'Podaję rzecz'

        with self:
            smach_rcprg.StateMachine.add('RememberCurrentPose', navigation.RememberCurrentPose(sim_mode, conversation_interface),
                                         transitions={'ok': 'SetHeightMid', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={'current_pose': 'initial_pose'})

            smach_rcprg.StateMachine.add('SetHeightMid', navigation.SetHeight(sim_mode, conversation_interface),
                                         transitions={'ok': 'SetKeeperPose', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={'torso_height': 'default_height'})

            smach_rcprg.StateMachine.add('SetKeeperPose', navigation.SetObjectPose(sim_mode, conversation_interface, kb_places),
                                         transitions={'ok': 'SetNavParams', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={})

            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(sim_mode),
                                         transitions={'ok': 'MoveToHuman', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={'max_lin_vel_in': 'max_lin_vel', 'max_lin_accel_in': 'max_lin_accel'})

            smach_rcprg.StateMachine.add('MoveToHuman', navigation.MoveToHumanComplex(sim_mode, conversation_interface, kb_places),
                                         transitions={'FINISHED': 'SayAskKeeperForGoods', 'PREEMPTED': 'PREEMPTED', 'FAILED': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={'goal': 'object_pose', 'susp_data': 'susp_data'})

            smach_rcprg.StateMachine.add('SayAskKeeperForGoods', SayAskKeeperForGoods(sim_mode, conversation_interface, input_keys),
                                         transitions={'ok': 'MoveBack', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                      'timeout': 'SayAskKeeperForGoods', 'shutdown': 'shutdown', 'unexpected_question': 'MoveBackAfterUnexpectedQuestion'},
                                         remapping={'q_load_answer_id': 'q_load_answer_id'})

            smach_rcprg.StateMachine.add('MoveBack', navigation.MoveToComplexBlocking(sim_mode, conversation_interface, kb_places),
                                         transitions={'FINISHED': 'TellInfoFromKeeper', 'PREEMPTED': 'PREEMPTED', 'FAILED': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={'goal': 'initial_pose'})
            
            smach_rcprg.StateMachine.add('MoveBackAfterUnexpectedQuestion', navigation.MoveToComplexBlocking(sim_mode, conversation_interface, kb_places),
                                            transitions={'FINISHED': 'KillTask', 'PREEMPTED': 'PREEMPTED', 'FAILED': 'FAILED',
                                                        'shutdown': 'shutdown'},
                                            remapping={'goal': 'initial_pose'})

            smach_rcprg.StateMachine.add('KillTask', KillTask(sim_mode, conversation_interface),
                                            transitions={'ok': 'FINISHED', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                        'shutdown': 'shutdown'})

            smach_rcprg.StateMachine.add('TellInfoFromKeeper', TellInfoFromKeeper(sim_mode, conversation_interface, input_keys),
                                         transitions={'restart': 'FINISHED', 'ok': 'SetHeightEnd', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                      'shutdown': 'shutdown', 'timeout': 'TellInfoFromKeeper', 'turn_around': 'TurnAroundB1', 'follow_up_answer': 'SetKeeperPose'},
                                         remapping={'q_load_answer_id': 'q_load_answer_id'})

            smach_rcprg.StateMachine.add('TurnAroundB1', navigation.RememberCurrentPose(sim_mode, conversation_interface),
                                         transitions={'ok': 'TurnAroundB2', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={'current_pose': 'current_pose'})

            smach_rcprg.StateMachine.add('TurnAroundB2', navigation.TurnAround(sim_mode, conversation_interface),
                                         transitions={'ok': 'TellInfoFromKeeper', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                      'shutdown': 'shutdown', 'stall': 'SayAskKeeperForGoods'},
                                         remapping={'current_pose': 'current_pose'})

            smach_rcprg.StateMachine.add('SetHeightEnd', navigation.SetHeight(sim_mode, conversation_interface),
                                         transitions={'ok': 'SayIFinished', 'preemption': 'PREEMPTED', 'error': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={'torso_height': 'default_height'})

            smach_rcprg.StateMachine.add('SayIFinished', SayIFinished(sim_mode, conversation_interface),
                                         transitions={'ok': 'FINISHED', 'shutdown': 'shutdown', 'preemption': 'PREEMPTED', 'error': 'FAILED'})
