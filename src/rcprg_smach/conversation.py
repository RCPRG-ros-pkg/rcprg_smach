#!/usr/bin/env python3
# encoding: utf8

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

import tiago_msgs.msg


class ConversationMachine(Node):
    def __init__(self, item_types, sim_mode):
        super().__init__('conversation_machine')
        self.__item_types__ = item_types
        self.__stop__ = False
        self.__intent_list__ = set()
        self.__intent_list_lock__ = threading.Lock()
        self.__sim_mode = sim_mode

        print('ConversationMachine.__init__: waiting for rico_says ActionServer...')
        if self.__sim_mode == 'real':
            self.rico_says_client = ActionClient(
                self, tiago_msgs.msg.SaySentence, 'rico_says')
            # In ROS2, you can use the service_is_ready() method to check if the server is available.
            while not self.rico_says_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().info("Waiting for the 'rico_says' ActionServer...")

        print('ConversationMachine.__init__: connected to rico_says ActionServer')

        self.sub = self.create_subscription(
            tiago_msgs.msg.Command, "rico_filtered_cmd", self.__callbackRicoCmd__, 10)
        self.pub = self.create_publisher(Bool, '/activate_vad', 10)

        # Expected queries
        self.__expected_query_types__ = set()
        self.__expected_queries__ = {}
        # self.__expected_autoremove_dict__ = {}

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
        self.__thread_hear__ = threading.Thread(
            target=self.__spin_hear__, args=(1,))
        self.__thread_hear__.start()

        self.__thread_speak__ = threading.Thread(
            target=self.__spin_speak__, args=(1,))
        self.__thread_speak__.start()

    def stop(self):
        print('ConversationMachine stopping...')
        self.__stop__ = True
        self.__thread_hear__.join()
        self.__thread_speak__.join()
        print('ConversationMachine stopping done.')

    def __spin_hear__(self, args):
        while not self.__stop__:
            self.__intent_list_lock__.acquire()
            for intent in self.__intent_list__:
                query_type = self.__getNameForIntent__(intent)
                # Manage expected queries
                if query_type in self.__expected_query_types__:
                    self.__expected_queries__[query_type] = intent

                # Manage automatic answers
                answer_id_list = self.getAutomaticAnswersIds(query_type)
                self.__pending_automatic_answers__ = self.__pending_automatic_answers__ + answer_id_list

                # Manage unexpected queries
                if not query_type in self.__expected_query_types__ and not bool(answer_id_list):
                    # Add special answer for unexpected intent
                    self.__pending_automatic_answers__.append(-1)

            # Remove all intents
            self.__intent_list__ = set()
            self.__intent_list_lock__.release()

            time.sleep(0.1)

    def __spin_speak__(self, args):
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
        assert isinstance(text, unicode)
        self.__automatic_answers_id_map__[
            self.__automatic_answer_id__] = (query_type, text)
        if not query_type in self.__automatic_answers_name_map__:
            self.__automatic_answers_name_map__[query_type] = {}
        self.__automatic_answers_name_map__[
            query_type][self.__automatic_answer_id__] = text
        self.__automatic_answer_id__ = self.__automatic_answer_id__ + 1
        return self.__automatic_answer_id__-1

    # Removes automatic answer, and waits for its completion if currently executed or queued
    def removeAutomaticAnswer(self, answer_id):
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
        if not query_type in self.__automatic_answers_name_map__:
            return []
        result = []
        for answer_id, text in self.__automatic_answers_name_map__[query_type].iteritems():
            result.append(answer_id)
        return result

    def __getAutomaticAnswerText__(self, answer_id):
        if answer_id == -1:
            return 'niekorzystne warunki pogodowe nie wiem o co chodzi'
        query_type, text = self.__automatic_answers_id_map__[answer_id]
        return text

    # Speaks a sentence and waits until it is finished
    def speakNowBlocking(self, text):
        print('Rico says (blocking): "' + text + '"')
        goal_msg = tiago_msgs.msg.SaySentence.Goal()
        goal_msg.sentence = text
        if self.__sim_mode == 'real':
            print('sending conversation goal')
            future = self.rico_says_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()

    def startListening(self):
        self.pub.publish(Bool())

    def addExpected(self, query_type):
        self.__intent_list_lock__.acquire()
        assert not query_type in self.__expected_query_types__
        self.__expected_query_types__.add(query_type)
        # self.__expected_autoremove_dict__[query_type] = autoremove
        self.__intent_list_lock__.release()

    def removeExpected(self, query_type):
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
