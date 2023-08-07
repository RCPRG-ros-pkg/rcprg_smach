#!/usr/bin/env python
# encoding: utf8

import threading
import time

import rospy
import smach
import smach_ros
import std_msgs

import unicodedata
import tiago_msgs.msg
import os

import random
import string

from std_msgs.msg import String
from task_database.srv import GetScenarioInputs, CloneScenarioWithNewIntent, AddParamsForScenario

import actionlib

#
# New, high-level interface for conversations
#
class SimpleConversationMachine:
    def __init__(self, scenario_id, intent_name, sim_mode=''):
        print 'SimpleConversationMachine args: ', scenario_id, intent_name
        self.scenario_id = scenario_id
        self.intent_name = intent_name
        rospy.wait_for_service('get_scenario_inputs')
        rospy.wait_for_service('clone_scenario_with_new_intent')
        rospy.wait_for_service('add_params_for_scenario')
        self.get_scenario_inputs = rospy.ServiceProxy('get_scenario_inputs', GetScenarioInputs)
        self.clone_scenario_with_new_intent = rospy.ServiceProxy('clone_scenario_with_new_intent', CloneScenarioWithNewIntent)
        self.add_params_for_scenario = rospy.ServiceProxy('add_params_for_scenario', AddParamsForScenario)
        self.__item_types__ = list(map(
            lambda scenario_input: scenario_input.intent,
            self.get_scenario_inputs(int(scenario_id)).inputs
        ))
        print self.__item_types__
        #@TODO: send it to lang processor
        self.__stop__ = False
        self.__intent_list__ = set()
        self.__intent_list_lock__ = threading.Lock()
        self.__sim_mode = sim_mode

        # self.dialogflow_agent = DialogflowAgent(
        #     agent_name, 'me4', cred_file_incare_dialog)

        print 'ConversationMachine.__init__: waiting for rico_says ActionServer...'
        # if self.__sim_mode == 'real':

        self.rico_says_client = actionlib.SimpleActionClient(
            'rico_says', tiago_msgs.msg.SaySentenceAction)
        self.rico_says_client.wait_for_server()

        print 'ConversationMachine.__init__: connected to rico_says ActionServer'

        self.sub = rospy.Subscriber(
            "rico_filtered_cmd", tiago_msgs.msg.Command, self.__callbackRicoCmd__)
        self.pub = rospy.Publisher(
            '/activate_vad', std_msgs.msg.Bool, queue_size=10)
        self.intents_pub = rospy.Publisher(
            '/new_intent', tiago_msgs.msg.NewIntent, queue_size=10)
        self.txt_pub = rospy.Publisher('/txt_send', String)

        # Expected queries
        self.__expected_intents__ = set()
        self.__got_expected_intents__ = {}
        #self.__expected_autoremove_dict__ = {}

        # Automatic answers
        self.__automatic_answer_id__ = 0
        self.__automatic_answers_name_map__ = {}
        self.__automatic_answers_id_map__ = {}
        self.__pending_automatic_answers__ = []
        self.__current_automatic_answer__ = None


    def __callbackRicoCmd__(self, data):
        # Data is of type tiago_msgs.msg.Command, i.e. it encapsulates an intent
        print 'UUU, some data received on /rico_filtered_cmd: ', data
        assert isinstance(data, tiago_msgs.msg.Command)
        self.__intent_list_lock__.acquire()
        self.__intent_list__.add(data)
        self.__intent_list_lock__.release()

    def start(self):
        print('CONVERSATION start')
        self.__thread_hear__ = threading.Thread(
            target=self.__spin_hear__, args=(1,))
        self.__thread_hear__.start()

        self.__thread_speak__ = threading.Thread(
            target=self.__spin_speak__, args=(1,))
        self.__thread_speak__.start()

    def stop(self):
        print 'ConversationMachine stopping...'
        self.__stop__ = True
        self.__thread_hear__.join()
        self.__thread_speak__.join()
        print 'ConversationMachine stopping done.'

    def __spin_hear__(self, args):
        print('CONVERSATION __spin_hear__')
        while not self.__stop__:
            self.__intent_list_lock__.acquire()
            for intent in self.__intent_list__:
                print 'CONVERSATION intent received: ', intent.intent_name

                intent_name = intent.intent_name

                if intent_name in self.__expected_intents__:
                    self.__got_expected_intents__[intent_name] = intent

                # Manage automatic answers
                answer_id_list = self.getAutomaticAnswersIds(intent_name)
                self.__pending_automatic_answers__ = self.__pending_automatic_answers__ + answer_id_list

                print self.__expected_intents__
                print answer_id_list

                # Manage unexpected queries
                if not intent_name in self.__expected_intents__ and not bool(answer_id_list):
                    print 'INTENT IS UNEXPECTED'
                    # Add special answer for unexpected intent
                    self.__pending_automatic_answers__.append(-1)

            # Remove all intents
            self.__intent_list__ = set()
            self.__intent_list_lock__.release()

            time.sleep(0.1)

    def __spin_speak__(self, args):
        print('CONVERSATION __spin_speak__')

        while not self.__stop__:
            # Manage automatic answers
            self.__intent_list_lock__.acquire()
            if self.__pending_automatic_answers__:
                answer_id = self.__pending_automatic_answers__.pop(0)
                self.__current_automatic_answer__ = answer_id
                self.__intent_list_lock__.release()
                self.speakNowBlocking(
                    self.__getAutomaticAnswerText__(answer_id))
                self.__intent_list_lock__.acquire()
                self.__current_automatic_answer__ = None
            self.__intent_list_lock__.release()

            time.sleep(0.1)

    def setAutomaticAnswer(self, intent, text):
        print 'CONVERSATION setAutomaticAnswer', intent, text

        assert isinstance(text, unicode)

        self.__automatic_answers_id_map__[
            self.__automatic_answer_id__] = (intent, text)
        if not intent in self.__automatic_answers_name_map__:
            self.__automatic_answers_name_map__[intent] = {}
        self.__automatic_answers_name_map__[
            intent][self.__automatic_answer_id__] = text
        self.__automatic_answer_id__ = self.__automatic_answer_id__ + 1
        print 'Successfully setAutomaticAnswer'
        return self.__automatic_answer_id__-1

    # Removes automatic answer, and waits for its completion if currently executed or queued
    def removeAutomaticAnswer(self, answer_id):
        print 'CONVERSATION removeAutomaticAnswer', answer_id

        assert isinstance(answer_id, (int, long))

        while True:
            break_loop = True
            self.__intent_list_lock__.acquire()
            if self.__current_automatic_answer__ == answer_id:
                break_loop = False
            else:
                for a_id in self.__pending_automatic_answers__:
                    if a_id == answer_id:
                        break_loop = False
                        break
            if break_loop:
                # The lock in not released here!
                break
            self.__intent_list_lock__.release()
            time.sleep(0.1)
        intent, text = self.__automatic_answers_id_map__[answer_id]
        del self.__automatic_answers_name_map__[intent][answer_id]
        del self.__automatic_answers_id_map__[answer_id]
        self.__intent_list_lock__.release()

    def getAutomaticAnswersIds(self, intent):
        print 'CONVERSATION getAutomaticAnswersIds', intent

        if not intent in self.__automatic_answers_name_map__:
            return []
        result = []
        for answer_id, text in self.__automatic_answers_name_map__[intent].iteritems():
            result.append(answer_id)
        return result

    def __getAutomaticAnswerText__(self, answer_id):
        print 'CONVERSATION __getAutomaticAnswerText__', answer_id

        if answer_id == -1:
            return 'niekorzystne warunki pogodowe nie wiem o co chodzi'
        intent, text = self.__automatic_answers_id_map__[answer_id]
        return text

    # Speaks a sentence and waits until it is finished
    def speakNowBlocking(self, text):
        print 'CONVERSATION speakNowBlocking', text

        print 'Rico says (blocking): "' + text + '"'
        goal = tiago_msgs.msg.SaySentenceGoal()
        goal.sentence = text
        # if self.__sim_mode == 'real': # can be 'gazebo' or 'real'

        print 'sending conversation goal'
        self.rico_says_client.send_goal(goal)
        self.rico_says_client.wait_for_result()

        print 'Rico says (blocking) finished'

    # Starts speaking a sentence, returns id for polling
    # def speakNowBlocking(self, text):
    #    print 'Rico says (non-blocking): "' + sentence + '"'
    #    goal = tiago_msgs.msg.SaySentenceGoal()
    #    goal.sentence = sentence
    #    self.rico_says_client.send_goal(goal)
    #    self.rico_says_client.wait_for_result()

    def addExpected(self, intent):
        print 'CONVERSATION addExpected', intent
        self.__intent_list_lock__.acquire()
        assert not intent in self.__expected_intents__
        self.__expected_intents__.add(intent)
        #self.__expected_autoremove_dict__[query_type] = autoremove
        self.__intent_list_lock__.release()

    def removeExpected(self, intent):
        print 'CONVERSATION removeExpected', intent
        self.__intent_list_lock__.acquire()
        assert intent in self.__expected_intents__
        # Consume, if there is any left
        if intent in self.__got_expected_intents__:
            del self.__got_expected_intents__[intent]
        self.__expected_intents__.remove(intent)
        self.__intent_list_lock__.release()

    def consumeExpected(self, intent):  # , remove):
        print 'CONVERSATION consumeExpected', intent  # , remove
        self.__intent_list_lock__.acquire()

        if intent in self.__got_expected_intents__:
            result = self.__got_expected_intents__[intent]
            del self.__got_expected_intents__[intent]
            self.__intent_list_lock__.release()
            return result
        else:
            self.__intent_list_lock__.release()
            return None

    def startListening(self):
        print('CONVERSATION startListening')
        self.pub.publish(std_msgs.msg.Bool())
