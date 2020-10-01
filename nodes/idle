#!/usr/bin/env python
# encoding: utf8

import sys
import time

import rospy
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
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        print 'MainState.execute'

        answer1_id = self.conversation_interface.setAutomaticAnswer( 'q_current_task', u'Nic nie robię' )
        answer2_id = self.conversation_interface.setAutomaticAnswer( 'q_load', u'niekorzystne warunki pogodowe nic nie wiozę' )

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

            rospy.sleep(0.1)
        raise Exception('Unreachable code')

#
# The SM that govenrs the highest-level behaviour.
#

class MainSM(smach_rcprg.StateMachine):
    def __init__(self):
        smach_rcprg.StateMachine.__init__(self, outcomes=['Finished', 'shutdown'])

        places_xml_filename = rospy.get_param('/kb_places_xml')
        sim_mode = str(rospy.get_param('/sim_mode'))
        assert sim_mode in ['sim', 'gazebo', 'real']

        self.conversation_interface = rcprg_smach.conversation.ConversationMachine([
                ('q_current_task', 'projects/incare-dialog-agent/agent/intents/8f45359d-ee47-4e10-a1b2-de3f3223e5b4'),
                ('q_load',        'projects/incare-dialog-agent/agent/intents/b8743ab9-08a1-49e8-a534-abb65155c507')
                #('ack',           'projects/incare-dialog-agent/agent/intents/ef92199b-d298-470c-8df3-1e1047dd70d1'),
                #('ack_i_took',    'projects/incare-dialog-agent/agent/intents/181621b6-e91e-4244-a925-c5dc32ee1f1b'),
                #('ack_i_gave',    'projects/incare-dialog-agent/agent/intents/d017cbd0-93f8-45b2-996e-043cdccab629'),
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

def main():
    da = DynAgent( 'idle' )
    da.run( MainSM() )

    return 0

if __name__ == '__main__':
    main()
