#!/usr/bin/env python
# encoding: utf8

import sys

import rospy
import smach
import smach_ros

import rcprg_smach.conversation
import rcprg_smach.bring_goods
import rcprg_smach.smach_rcprg as smach_rcprg
from rcprg_smach.dynamic_agent import DynAgent

import rcprg_kb.places_xml as kb_p

from rcprg_smach.task_manager import PoseDescription

from pl_nouns.dictionary_client import DisctionaryServiceClient

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

        if len(sys.argv) < 3:
            raise Exception('Too few arguments: ' + str(sys.argv))

        goods_name = None
        for idx in range(1, len(sys.argv), 2):
            if sys.argv[idx] == 'przedmiot':
                goods_name = sys.argv[idx+1]

        if goods_name is None:
            raise Exception('Argument "goods_name" is missing in argv: ' + str(sys.argv))        

        if isinstance(goods_name, str):
            goods_name = goods_name.decode('utf-8')
        goods_name = goods_name.encode('utf-8').decode('utf-8')

        dictionary = DisctionaryServiceClient()
        goods_name_m = dictionary.getCases(goods_name).getCase('mianownik')

        self.conversation_interface = rcprg_smach.conversation.ConversationMachine([
                ('ack',             'projects/incare-dialog-agent/agent/intents/ef92199b-d298-470c-8df3-1e1047dd70d1'),
                ('ack_i_took',      'projects/incare-dialog-agent/agent/intents/181621b6-e91e-4244-a925-c5dc32ee1f1b'),
                ('ack_i_gave',      'projects/incare-dialog-agent/agent/intents/d017cbd0-93f8-45b2-996e-043cdccab629'),
                ('q_current_task',  'projects/incare-dialog-agent/agent/intents/8f45359d-ee47-4e10-a1b2-de3f3223e5b4'),
                ('q_load',          'projects/incare-dialog-agent/agent/intents/b8743ab9-08a1-49e8-a534-abb65155c507'),
                ('turn_around',     'projects/incare-dialog-agent/agent/intents/b4cb9f2e-2589-44dd-af14-a8f899c40ec0'),
            ],sim_mode)
        self.conversation_interface.start()

        self.userdata.goal = goods_name_m

        with self:
            smach_rcprg.StateMachine.add('BringGoods',
                                    rcprg_smach.bring_goods.BringGoods(sim_mode, self.conversation_interface, kb_places),
                                    transitions={'FINISHED':'Cleanup', 'PREEMPTED':'Cleanup', 'FAILED': 'Cleanup',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'goal'})

            smach_rcprg.StateMachine.add('Cleanup',
                                    Cleanup(self.conversation_interface),
                                    transitions={'ok':'Finished', 'shutdown':'shutdown'},
                                    remapping={ })


    def shutdownRequest(self):
        self.conversation_interface.stop()
        self.request_preempt()

def main():
    da = DynAgent( 'bring_goods' )
    da.run( MainSM() )

    return 0

if __name__ == '__main__':
    main()
