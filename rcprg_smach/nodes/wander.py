#!/usr/bin/env python3
# encoding: utf8

import sys

import rospy
import smach
import smach_ros

import rcprg_smach.conversation
import rcprg_smach.wander
import rcprg_smach.smach_rcprg as smach_rcprg
from rcprg_smach.dynamic_agent import DynAgent

import rcprg_kb.places_xml as kb_p

from rcprg_smach.task_manager import PoseDescription

import pl_nouns.odmiana as ro


class Cleanup(smach_rcprg.State):
    def __init__(self, conversation_interface):
        self.conversation_interface = conversation_interface

        smach_rcprg.State.__init__(self, outcomes=['ok', 'shutdown'])

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        print 'Cleanup.execute'
        self.conversation_interface.stop()
        return 'ok'

#
# The SM that govenrs the highest-level behaviour.
#

class MainSM(smach_rcprg.StateMachine):
    def __init__(self):
        smach_rcprg.StateMachine.__init__(self, outcomes=['Finished', 'shutdown'])

        places_xml_filename = rospy.get_param('/kb_places_xml')
        sim_mode = str(rospy.get_param('/sim_mode'))
        assert sim_mode in ['sim', 'gazebo', 'real']

        print 'Reading KB for places from file "' + places_xml_filename + '"'
        kb_places = kb_p.PlacesXmlParser(places_xml_filename).getKB()

        # Create conversation interface that is shared between all states and SMs.
        self.conversation_interface = rcprg_smach.conversation.ConversationMachine([
                ('q_current_task', 'projects/incare-dialog-agent/agent/intents/8f45359d-ee47-4e10-a1b2-de3f3223e5b4'),
            ],sim_mode)
        self.conversation_interface.start()

        with self:
            smach_rcprg.StateMachine.add('Wander',
                                    rcprg_smach.wander.Wander(sim_mode, self.conversation_interface, kb_places),
                                    transitions={'FINISHED':'Cleanup', 'PREEMPTED':'Cleanup', 'FAILED': 'Cleanup',
                                    'shutdown':'shutdown'},
                                    remapping={ })

            smach_rcprg.StateMachine.add('Cleanup',
                                    Cleanup(self.conversation_interface),
                                    transitions={'ok':'Finished', 'shutdown':'shutdown'},
                                    remapping={ })

    def shutdownRequest(self):
        self.conversation_interface.stop()
        self.request_preempt()

def main():
    da = DynAgent( 'wander' )
    da.run( MainSM() )

    return 0

if __name__ == '__main__':
    main()
