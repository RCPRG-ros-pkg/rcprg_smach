#!/usr/bin/env python
# encoding: utf8

import sys
import time
import threading

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from rcprg_smach.ros_node_utils import get_node_names
import tiago_msgs.msg

class SmachShutdownManager:
    def __init__(self, main_sm, list_asw, list_sis):
        self.__main_sm__ = main_sm
        self.__list_asw__ = list_asw
        self.__list_sis__ = list_sis

    def on_shutdown(self):
        # Stop all introspection servers
        print 'SmachShutdownManager.on_shutdown: stopping all introspection servers'
        for sis in reversed(self.__list_sis__):
            sis.stop()

        # Stop all smach_ros action servers
        print 'SmachShutdownManager.on_shutdown: stopping all smach_ros action servers'
        for asw in reversed(self.__list_asw__):
            # This is a veru ugly hack:
            asw._action_server.action_server.started = False

            asw.wrapped_container.request_preempt()

        # Stop the main SM
        print 'SmachShutdownManager.on_shutdown: stopping the main SM'
        self.__main_sm__.request_preempt()

        #rospy.sleep(2.0)

class DynAgent:
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name)
        rospy.sleep(0.1)
        self.sub_task_state_cmd = rospy.Subscriber('/' + self.name + '/task_state_cmd', String, self.callbackTaskStateCmd)
        self.finished = False

        self.pub_diag = rospy.Publisher('/current_dyn_agent/diag', tiago_msgs.msg.DynAgentDiag, queue_size=10)

    def run(self, main_sm):
        self.main_sm = main_sm

        #sis_main = smach_ros.IntrospectionServer('behaviour_server', self.main_sm, '/SM_BEHAVIOUR_SERVER')
        #sis_main.start()

        #ssm = SmachShutdownManager(self.main_sm, [], [sis_main])
        ssm = SmachShutdownManager(self.main_sm, [], [])

        # Set shutdown hook
        rospy.on_shutdown( ssm.on_shutdown )

        thread_conn = threading.Thread(target=self.connectionCheckThread, args=(1,))
        thread_conn.start()

        smach_thread = threading.Thread(target=self.main_sm.execute)
        smach_thread.start()

        print 'Smach thread is running'

        # Block until everything is preempted
        smach_thread.join()
        self.finished = True
        thread_conn.join()

        print 'Smach thread is finished'
        ssm.on_shutdown()

    def callbackTaskStateCmd(self, data):
        print 'DynAgent.callback'
        if data.data == 'abort':
            print 'DynAgent "' + self.name + '" received abort command'
            self.finished = True
            self.main_sm.shutdownRequest()

    def getActiveStates(self, sm):
        result = []
        if hasattr( sm, 'get_active_states' ):
            for state_name in sm.get_active_states():
                st = sm.get_children()[state_name]
                if hasattr( st, 'description' ) and not st.description is None:
                    description = st.description
                else:
                    description = ''
                result.append( (state_name, description) )
            for state_name in sm.get_active_states():
                st = sm.get_children()[state_name]
                result = result + self.getActiveStates( st )
        return result

    def connectionCheckThread(self, args):
        while not self.finished:
            active_ros_nodes = get_node_names()
            if not '/task_harmonizer' in active_ros_nodes:
                print 'DynAgent "' + self.name + '" received has detected the task_harmonizer is dead'
                self.finished = True
                self.main_sm.shutdownRequest()
                break

            try:
                # Publish diagnostic information
                diag = tiago_msgs.msg.DynAgentDiag()
                diag.agent_name = self.name
                active_states = self.getActiveStates( self.main_sm )
                for state_name, state_desc in active_states:
                    diag.current_states.append( state_name )
                    diag.descriptions.append( state_desc )
                self.pub_diag.publish( diag )
            except Exception as e:
                print 'Detected exception in dynamic agent'
                print e
                self.finished = True
                self.main_sm.shutdownRequest()
                break

            time.sleep(0.2)
