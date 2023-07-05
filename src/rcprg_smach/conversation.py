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

# from dialogflow_agent.interfaces.DialogflowAgent import DialogflowAgent

import actionlib


# def remove_spaces(string):
#     result = string.replace(' ', '_')

#     return result


# def remove_diacritics(string):
#     # print string, type(string)
#     u_text = unicodedata.normalize('NFKD', unicode(string, 'utf-8'))
#     result = u_text.encode('ascii', 'ignore')

#     return result

# agent_name = 'robot-rico-qrct'
# cred_file_incare_dialog = os.environ['GOOGLE_CONVERSATIONAL_DIALOGFLOW']


#
# New, high-level interface for conversations
#
class ConversationMachine:
    # item_types is [(query_name, intent_name)]
    def __init__(self, item_types=[], sim_mode='', scenario_id=None, intent_name=None):
        print 'ConversationMachine args: ', item_types, sim_mode, scenario_id
        self.scenario_id = scenario_id
        self.intent_name = intent_name
        rospy.wait_for_service('get_scenario_inputs')
        rospy.wait_for_service('clone_scenario_with_new_intent')
        rospy.wait_for_service('add_params_for_scenario')
        self.get_scenario_inputs = rospy.ServiceProxy(
            'get_scenario_inputs', GetScenarioInputs)
        self.clone_scenario_with_new_intent = rospy.ServiceProxy('clone_scenario_with_new_intent', CloneScenarioWithNewIntent)
        self.add_params_for_scenario = rospy.ServiceProxy('add_params_for_scenario', AddParamsForScenario)
        self.__item_types__ = item_types if scenario_id is None else list(map(lambda scenario_input: (
            scenario_input.alias, scenario_input.intent), self.get_scenario_inputs(int(scenario_id)).inputs))
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
        self.__expected_query_types__ = set()
        self.__expected_queries__ = {}
        #self.__expected_autoremove_dict__ = {}

        # Automatic answers
        self.__automatic_answer_id__ = 0
        self.__automatic_answers_name_map__ = {}
        self.__automatic_answers_id_map__ = {}
        self.__pending_automatic_answers__ = []
        self.__current_automatic_answer__ = None


    def __callbackRicoCmd__(self, data):
        # Data is of type tiago_msgs.msg.Command, i.e. it encapsulates an intent
        assert isinstance(data, tiago_msgs.msg.Command)
        self.__intent_list_lock__.acquire()
        self.__intent_list__.add(data)
        self.__intent_list_lock__.release()

    def __getNameForIntent__(self, intent):
        assert isinstance(intent, tiago_msgs.msg.Command)
        for name, in_name in self.__item_types__:
            if in_name == intent.intent_name:
                return name
        return None

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
                print self.__intent_list__
                query_type = self.__getNameForIntent__(intent)
                # Manage expected queries
                if query_type in self.__expected_query_types__:
                    self.__expected_queries__[query_type] = intent

                # Manage automatic answers
                answer_id_list = self.getAutomaticAnswersIds(query_type)
                self.__pending_automatic_answers__ = self.__pending_automatic_answers__ + answer_id_list

                print self.__expected_query_types__
                print answer_id_list

                # Manage unexpected queries
                if not query_type in self.__expected_query_types__ and not bool(answer_id_list):
                    print 'QUERY IS UNEXPECTED'
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

    def setAutomaticAnswer(self, query_type, text):
        print 'CONVERSATION setAutomaticAnswer', query_type, text

        assert isinstance(text, unicode)

        self.__automatic_answers_id_map__[
            self.__automatic_answer_id__] = (query_type, text)
        if not query_type in self.__automatic_answers_name_map__:
            self.__automatic_answers_name_map__[query_type] = {}
        self.__automatic_answers_name_map__[
            query_type][self.__automatic_answer_id__] = text
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
        query_type, text = self.__automatic_answers_id_map__[answer_id]
        del self.__automatic_answers_name_map__[query_type][answer_id]
        del self.__automatic_answers_id_map__[answer_id]
        self.__intent_list_lock__.release()

    def getAutomaticAnswersIds(self, query_type):
        print 'CONVERSATION getAutomaticAnswersIds', query_type

        if not query_type in self.__automatic_answers_name_map__:
            return []
        result = []
        for answer_id, text in self.__automatic_answers_name_map__[query_type].iteritems():
            result.append(answer_id)
        return result

    def __getAutomaticAnswerText__(self, answer_id):
        print 'CONVERSATION __getAutomaticAnswerText__', answer_id

        if answer_id == -1:
            return 'niekorzystne warunki pogodowe nie wiem o co chodzi'
        query_type, text = self.__automatic_answers_id_map__[answer_id]
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

    def addExpected(self, query_type):
        print 'CONVERSATION addExpected', query_type
        self.__intent_list_lock__.acquire()
        assert not query_type in self.__expected_query_types__
        self.__expected_query_types__.add(query_type)
        #self.__expected_autoremove_dict__[query_type] = autoremove
        self.__intent_list_lock__.release()

    def removeExpected(self, query_type):
        print 'CONVERSATION removeExpected', query_type
        self.__intent_list_lock__.acquire()
        assert query_type in self.__expected_query_types__
        # Consume, if there is any left
        if query_type in self.__expected_queries__:
            del self.__expected_queries__[query_type]
        self.__expected_query_types__.remove(query_type)
        self.__intent_list_lock__.release()

    def consumeExpected(self, query_type):  # , remove):
        self.__intent_list_lock__.acquire()
        if query_type in self.__expected_queries__:
            result = self.__expected_queries__[query_type]
            del self.__expected_queries__[query_type]
            # if remove == True:
            #    self.__expected_query_types__.remove(query_type)
            self.__intent_list_lock__.release()
            return result
        else:
            self.__intent_list_lock__.release()
            return None

    def startListening(self):
        print('CONVERSATION startListening')
        self.pub.publish(std_msgs.msg.Bool())

    # def setContext(self, key, value):
    #     print 'setContext', key, value
    #     self.__context__[key] = value

    # def getContext(self, key):
    #     if key in self.__context__:
    #         return self.__context__[key]
    #     else:
    #         return None

    # def clearContext(self):
    #     self.__context__ = {}

    # def unexpected_question(self, original_query, question_text, default_intent_params=[]):
    #     param_name = 'question_' + remove_spaces(remove_diacritics(question_text))
    #     parameters = [{'name': param_name, 'prompt': question_text + '?'}]

    #     curr_intent = self.dialogflow_agent.get_intent(self.intent_name)

    #     is_current_intent_autogenerated = self.dialogflow_agent.is_autogenerated(curr_intent)

    #     if is_current_intent_autogenerated:
    #         self.dialogflow_agent.update_intent_with_parameters(curr_intent, parameters)
    #         self.add_params_for_scenario(int(self.scenario_id), list(map(lambda param: param['name'], parameters)))
    #     else:
    #         intent_display_name = original_query
    #         trigger_phrases = [original_query]

    #         autoresponses = ['dziękuję']
    #         response = self.dialogflow_agent.create_intent(
    #             intent_display_name, True, trigger_phrases, autoresponses, parameters, default_intent_params)

    #         self.clone_scenario_with_new_intent(int(self.scenario_id), response.name, list(
    #             map(lambda param: param.display_name, response.parameters)))


    # def trigger_dialogflow_with_text(self, text):
    #     self.txt_pub.publish(text)
