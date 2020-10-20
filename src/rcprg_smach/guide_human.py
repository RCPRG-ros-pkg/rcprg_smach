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

class SetHumanAndDestination(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['human_name', 'guide_destination'], output_keys=['human_pose', 'dest_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Znajduję człowieka i cel'
        self.kb_places = kb_places
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Ustalam gdzie jest człowiek i jego cel' )

        if isinstance(userdata.human_name, str):
            human_name = userdata.human_name.decode('utf-8')
        human_name = userdata.human_name.encode('utf-8').decode('utf-8')

        if isinstance(userdata.guide_destination, str):
            guide_destination = userdata.guide_destination.decode('utf-8')
        guide_destination = userdata.guide_destination.encode('utf-8').decode('utf-8')

        userdata.dest_pose = navigation.PoseDescription({'place_name':unicode(guide_destination)})
        places_ids = self.kb_places.getPointPlacesIds()
        place_name = unicode(human_name + "_awaiting")
        if unicode(human_name) in places_ids:
            userdata.human_pose = navigation.PoseDescription({'place_name':unicode(human_name)})
        else:
            self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Nie mam pozycji '+human_name+ u' w bazie wiedzy')
            return 'error'
        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class IntroduceTask(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, greeted):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['human_name', 'guide_destination'], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'timeout', 'shutdown'])
        print "INTRO INIT"
        self.conversation_interface = conversation_interface

        self.description = u'Nawiązuję interakcję'
        self.greeted = greeted
    def transition_function(self, userdata):
        print "INTRO EXE"

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
        if isinstance(userdata.guide_destination, str):
            guide_destination = userdata.guide_destination.decode('utf-8')
        guide_destination = userdata.guide_destination.encode('utf-8').decode('utf-8')

        dictionary = DisctionaryServiceClient()
        guide_dest_b = dictionary.getCases(guide_destination).getCase('dopelniacz')
        print "LAST GREEEETED: ", self.greeted.get()
        if not self.greeted.get():
            self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Dzień dobry, '+gender+u' udać się do ' + guide_dest_b + u', proszę podążać za mną.' )
        else:
            self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Przepraszam za przerwanie, już możemy kontynuować' )

        if self.__shutdown__:
            return 'shutdown'
        self.greeted.set(True)
        return 'ok'

class Goodbye(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['human_name', 'guide_destination'], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'timeout', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Żegnam się'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        gender = ""
        if isinstance(userdata.human_name, str):
            userdata.human_name = userdata.human_name.decode('utf-8')
        userdata.human_name = userdata.human_name.encode('utf-8').decode('utf-8')

        if userdata.human_name in ["John", "Peter"]:
            gender = "powinien Pan"
        else:
            gender = "powinna Pani"
        if isinstance(userdata.guide_destination, str):
            userdata.guide_destination = userdata.guide_destination.decode('utf-8')
        userdata.guide_destination = userdata.guide_destination.encode('utf-8').decode('utf-8')

        dictionary = DisctionaryServiceClient()
        guide_dest_b = dictionary.getCases(userdata.guide_destination).getCase('dopelniacz')

        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Dotarliśmy do '+ guide_dest_b + u' dowidzenia.' )

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'



class SayAskForGoods(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['goods_name'], output_keys=['q_load_answer_id'],
                             outcomes=['ok', 'preemption', 'timeout', 'error', 'shutdown', 'turn_around'])

        self.conversation_interface = conversation_interface

        self.description = u'Proszę o podanie rzeczy'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        assert isinstance(userdata.goods_name, unicode)

        goods_name = userdata.goods_name

        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe podaj mi {"' + goods_name + u'", biernik} i potwierdź' )

        self.conversation_interface.addExpected('ack')
        self.conversation_interface.addExpected('ack_i_gave')
        self.conversation_interface.addExpected('turn_around')

        answer_id = self.conversation_interface.setAutomaticAnswer( 'q_current_task', u'niekorzystne warunki pogodowe czekam na położenie {"' + goods_name + u'", dopelniacz}' )

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
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_gave')
                self.conversation_interface.removeExpected('turn_around')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                return 'timeout'

            if self.preempt_requested():
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_gave')
                self.conversation_interface.removeExpected('turn_around')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                self.service_preempt()
                return 'preemption'

            if self.conversation_interface.consumeExpected('ack') or\
                    self.conversation_interface.consumeExpected('ack_i_gave'):
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_gave')
                self.conversation_interface.removeExpected('turn_around')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                answer_id = self.conversation_interface.setAutomaticAnswer( 'q_load', u'niekorzystne warunki pogodowe wiozę {"' + goods_name + u'", biernik}' )
                userdata.q_load_answer_id = answer_id
                return 'ok'
            if self.conversation_interface.consumeExpected('turn_around'):
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_gave')
                self.conversation_interface.removeExpected('turn_around')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                return 'turn_around'

            rospy.sleep(0.1)

        raise Exception('Unreachable code')

class SayTakeGoods(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.TaskER.BlockingState.__init__(self, input_keys=['goods_name', 'q_load_answer_id'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown', 'timeout', 'turn_around'])

        self.conversation_interface = conversation_interface
        self.description = u'Proszę o odebranie rzeczy'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        assert isinstance(userdata.goods_name, unicode)

        goods_name = userdata.goods_name

        #self.conversation_interface.addSpeakSentence( u'Odbierz {"' + goods_name + u'", biernik} i potwierdź' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe odbierz {"' + goods_name + u'", biernik} i potwierdź' )

        self.conversation_interface.addExpected('ack')
        self.conversation_interface.addExpected('ack_i_took')
        self.conversation_interface.addExpected('turn_around')

        answer_id = self.conversation_interface.setAutomaticAnswer( 'q_current_task', u'niekorzystne warunki pogodowe czekam na odebranie {"' + goods_name + u'", dopelniacz}' )

        start_time = rospy.Time.now()
        while True:
            end_time = rospy.Time.now()
            loop_time = end_time - start_time
            loop_time_s = loop_time.secs

            if self.__shutdown__:
                return 'shutdown'

            if loop_time_s > ACK_WAIT_MAX_TIME_S:
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_took')
                self.conversation_interface.removeExpected('turn_around')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                # Do not remove q_load_answer, because we want to enter this state again
                return 'timeout'

            if self.preempt_requested():
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_took')
                self.conversation_interface.removeExpected('turn_around')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                if not userdata.q_load_answer_id is None:
                    self.conversation_interface.removeAutomaticAnswer(userdata.q_load_answer_id)
                self.service_preempt()
                return 'preemption'

            if self.conversation_interface.consumeExpected('ack') or\
                    self.conversation_interface.consumeExpected('ack_i_took'):
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_took')
                self.conversation_interface.removeExpected('turn_around')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                if not userdata.q_load_answer_id is None:
                    self.conversation_interface.removeAutomaticAnswer(userdata.q_load_answer_id)
                return 'ok'
            if self.conversation_interface.consumeExpected('turn_around'):
                self.conversation_interface.removeExpected('ack')
                self.conversation_interface.removeExpected('ack_i_took')
                self.conversation_interface.removeExpected('turn_around')
                self.conversation_interface.removeAutomaticAnswer(answer_id)
                return 'turn_around'

            rospy.sleep(0.1)

        raise Exception('Unreachable code')

class SayIFinished(smach_rcprg.TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.TaskER.BlockingState.__init__(self,
                             outcomes=['ok', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Mówię, że zakończyłem'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        #self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe zakończyłem zadanie' )

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class GuideHuman(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places,greeted):
        smach_rcprg.StateMachine.__init__(self, input_keys=['human_name','guide_destination','susp_data'], output_keys=['susp_data'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5
        # TODO: use knowledge base for this:
        self.userdata.default_height = 0.2
        self.userdata.lowest_height = 0.0

        self.description = u'Podaję rzecz'
        self.greeted = greeted
        with self:
            smach_rcprg.StateMachine.add('SetHeightMid', navigation.SetHeight(sim_mode, conversation_interface),
                                    transitions={'ok':'SetHumanAndDestination', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'torso_height':'default_height'})

            smach_rcprg.StateMachine.add('SetHumanAndDestination', SetHumanAndDestination(sim_mode, conversation_interface, kb_places),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(sim_mode),
                                    transitions={'ok':'MoveToHuman', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach_rcprg.StateMachine.add('MoveToHuman', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'IntroduceTask', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'human_pose', 'susp_data':'susp_data'})

            smach_rcprg.StateMachine.add('IntroduceTask', IntroduceTask(sim_mode, conversation_interface, self.greeted),
                                    transitions={'ok':'MoveToDestination', 'preemption':'PREEMPTED', 'error':'FAILED',
                                    'timeout':'IntroduceTask','shutdown':'shutdown', },
                                    remapping={'human_name':'human_name', 'guide_destination':'guide_destination'})

            smach_rcprg.StateMachine.add('MoveToDestination', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'Goodbye', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'dest_pose', 'susp_data':'susp_data'})

            smach_rcprg.StateMachine.add('Goodbye', Goodbye(sim_mode, conversation_interface),
                                    transitions={'ok':'SayIFinished', 'preemption':'PREEMPTED', 'error':'FAILED',
                                    'timeout':'SayIFinished','shutdown':'shutdown', },
                                    remapping={'human_name':'human_name', 'guide_destination':'guide_destination'})

            # smach_rcprg.StateMachine.add('SetHeightLow', navigation.SetHeight(sim_mode, conversation_interface),
            #                         transitions={'ok':'AskForGoods', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'torso_height':'lowest_height'})

            # smach_rcprg.StateMachine.add('AskForGoods', SayAskForGoods(sim_mode, conversation_interface),
            #                         transitions={'ok':'MoveToDestination', 'preemption':'PREEMPTED', 'error':'FAILED',
            #                         'timeout':'AskForGoods','shutdown':'shutdown', 'turn_around':'TurnAroundA1'},
            #                         remapping={'goods_name':'goal', 'q_load_answer_id':'q_load_answer_id'})

            # smach_rcprg.StateMachine.add('MoveBack', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
            #                         transitions={'FINISHED':'SayTakeGoods', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'goal':'initial_pose', 'susp_data':'susp_data'})

            # smach_rcprg.StateMachine.add('SayTakeGoods', SayTakeGoods(sim_mode, conversation_interface),
            #                         transitions={'ok':'SetHeightEnd', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown', 'timeout':'SayTakeGoods', 'turn_around':'TurnAroundB1'},
            #                         remapping={'goods_name':'goal', 'q_load_answer_id':'q_load_answer_id'})

            # smach_rcprg.StateMachine.add('TurnAroundA1', navigation.RememberCurrentPose(sim_mode),
            #                         transitions={'ok':'TurnAroundA2', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'current_pose':'current_pose'})

            # smach_rcprg.StateMachine.add('TurnAroundA2', navigation.TurnAround(sim_mode, conversation_interface),
            #                         transitions={'ok':'AskForGoods', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown', 'stall':'AskForGoods'},
            #                         remapping={'current_pose':'current_pose'})

            # smach_rcprg.StateMachine.add('TurnAroundB1', navigation.RememberCurrentPose(sim_mode),
            #                         transitions={'ok':'TurnAroundB2', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'current_pose':'current_pose'})

            # smach_rcprg.StateMachine.add('TurnAroundB2', navigation.TurnAround(sim_mode, conversation_interface),
            #                         transitions={'ok':'SayTakeGoods', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown', 'stall':'AskForGoods'},
            #                         remapping={'current_pose':'current_pose'})

            # smach_rcprg.StateMachine.add('SetHeightEnd', navigation.SetHeight(sim_mode, conversation_interface),
            #                         transitions={'ok':'SayIFinished', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'torso_height':'default_height'})

            smach_rcprg.StateMachine.add('SayIFinished', SayIFinished(sim_mode, conversation_interface),
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
