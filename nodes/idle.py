#!/usr/bin/env python3
# encoding: utf8

import sys
import time

import rclpy
from rclpy.node import Node

import smach
import smach_ros

import rcprg_smach.conversation
import rcprg_smach.navigation
import rcprg_smach.smach_rcprg as smach_rcprg
from rcprg_smach.dynamic_agent import DynAgent

import rcprg_kb.places_xml as kb_p

from rcprg_smach.task_manager import PoseDescription

class MainState(smach_rcprg.State):
    def __init__(self, conversation_interface):
        self.conversation_interface = conversation_interface
        smach_rcprg.State.__init__(self, outcomes=['shutdown', 'preemption'])

    def execute(self, userdata):
        rclpy.logging.get_logger(self.__class__.__name__).info(f'Executing state: {self.__class__.__name__}')
        print('MainState.execute')

        answer1_id = self.conversation_interface.setAutomaticAnswer('q_current_task', u'Nic nie robię')
        answer2_id = self.conversation_interface.setAutomaticAnswer('q_load', u'niekorzystne warunki pogodowe nic nie wiozę')

        while True:
            if self.preempt_requested():
                self.conversation_interface.removeAutomaticAnswer(answer1_id)
                self.conversation_interface.removeAutomaticAnswer(answer2_id)
                self.conversation_interface.stop()
                self.service_preempt()
                return 'preemption'

            if self.__shutdown__:
                self.conversation_interface.removeAutomaticAnswer(answer1_id)
                self.conversation_interface.removeAutomaticAnswer(answer2_id)
                self.conversation_interface.stop()
                self.service_preempt()
                return 'shutdown'

            time.sleep(0.1)
        raise Exception('Unreachable code')

class MainSM(smach_rcprg.StateMachine):
    def __init__(self):
        super().__init__(outcomes=['Finished', 'shutdown'])
        
        node = rclpy.create_node('MainSM_node')
        places_xml_filename = node.get_parameter('/kb_places_xml').value
        sim_mode = str(node.get_parameter('/sim_mode').value)
        assert sim_mode in ['sim', 'gazebo', 'real']

        self.conversation_interface = rcprg_smach.conversation.ConversationMachine([
                ('q_current_task', 'projects/incare-dialog-agent/agent/intents/8f45359d-ee47-4e10-a1b2-de3f3223e5b4'),
                ('q_load',        'projects/incare-dialog-agent/agent/intents/b8743ab9-08a1-49e8-a534-abb65155c507')
            ],sim_mode)
        self.conversation_interface.start()

        with self:
            smach_rcprg.StateMachine.add('MainState',
                                    MainState(self.conversation_interface),
                                    transitions={'preemption':'Finished', 'shutdown':'shutdown'},
                                    remapping={ })

    def shutdownRequest(self):
        self.conversation_interface.stop()
        self.request_preempt()

def main(args=None):
    rclpy.init(args=args)
    da = DynAgent('idle')
    da.run(MainSM())
    rclpy.shutdown()

    return 0

if __name__ == '__main__':
    main()