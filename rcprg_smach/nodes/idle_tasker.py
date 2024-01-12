#!/usr/bin/env python3
# encoding: utf8

import sys
import rclpy
from rclpy.node import Node
import smach
import smach_ros

import rcprg_smach.conversation
from TaskER.dynamic_agent import DynAgent

# import tiago_kb.places_xml as kb_p

from rcprg_smach.task_manager import PoseDescription

# from pl_nouns.dictionary_client import DisctionaryServiceClient

from TaskER.TaskER import TaskER
from tasker_msgs.msg import RobotResource, ScheduleParams
from tasker_msgs.srv import SuspendConditionsRequest, SuspendConditionsResponse
from tasker_msgs.srv import CostConditionsRequest, CostConditionsResponse
# for ExeSuspension state
import subprocess
import imp

my_name = None

global USE_SMACH_INRTOSPECTION_SERVER
USE_SMACH_INRTOSPECTION_SERVER = False


class MainState(TaskER.SuspendableState):
    def __init__(self):
        TaskER.SuspendableState.__init__(
            self, outcomes=['shutdown', 'PREEMPTED'])

    def transition_function(self, userdata):
        node = rclpy.create_node(self.__class__.__name__)
        node.get_logger().info('{}: Executing state: {}'.format(
            node.get_name(), self.__class__.__name__))
        print('MainState.execute')

        # answer1_id = self.conversation_interface.setAutomaticAnswer( 'q_current_task', u'Nic nie robię' )
        # answer2_id = self.conversation_interface.setAutomaticAnswer( 'q_load', u'niekorzystne warunki pogodowe nic nie wiozę' )

        while True:
            if self.is_suspension_flag() is not None:
                print("\n" * 4 + "IDLE GOT SUSP FLAG" + "\n" * 4)

                if self.__shutdown__:
                    print("IDLE GOT SHUTDOWN FLAG")
                    self.request_preempt()
                    return 'shutdown'

            rclpy.spin_once(node, timeout_sec=0.2)
        raise Exception('Unreachable code')


def ptf_csp(ptf_id):
    global my_name
    node = rclpy.create_node('ptf_csp_node')
    flag = None
    self_terminate_flag = node.get_parameter('/term_ID').value
    cost = node.get_parameter("/cost_ID_" + my_name).value
    print("self_terminate_flag: ", self_terminate_flag)
    if self_terminate_flag == True:
        flag = 'self-terminate'
    if ptf_id[0] == "scheduleParams":
        return [flag, ScheduleParams(cost=cost, completion_time=1000, cost_per_sec=0, final_resource_state=RobotResource())]
    elif ptf_id[0] == "suspendCondition":
        req = SuspendConditionsRequest()
        req = ptf_id[1]
        return [flag, SuspendConditionsResponse(cost_per_sec=0, cost_to_resume=0)]
    elif ptf_id[0] == "startCondition":
        req = CostConditionsRequest()
        req = ptf_id[1]
        return [flag, CostConditionsResponse(cost_per_sec=0, cost_to_complete=0)]


class MyTaskER(TaskER):
    def __init__(self, da_state_name, da_name):
        global USE_SMACH_INRTOSPECTION_SERVER
        global my_name
        my_name = str(da_name)
        self.name = my_name
        rclpy.init()
        self.node = rclpy.create_node(self.name)
        self.sim_mode = None
        self.conversation_interface = None
        self.kb_places = None
        TaskER.__init__(self, da_state_name)
        if USE_SMACH_INRTOSPECTION_SERVER:
            self.sis = smach_ros.IntrospectionServer(
                unicode(str("/"+self.name+"smach_view_server")), self, unicode(self.name))
            self.sis.start()
        self.my_fsm = MainState()

    def shutdownRequest(self):
        global USE_SMACH_INRTOSPECTION_SERVER
        print("my-tasker -----------------------   shutdown")

        # self.conversation_interface.stop()

        if USE_SMACH_INRTOSPECTION_SERVER:
            self.sis.stop()
            self.sis.clear()
        self.request_preempt()

    def cleanup_tf(self):
        rclpy.logging.get_logger(self.__class__.__name__).info(
            f'Executing state: {self.__class__.__name__}')
        print('Cleanup.execute')
        # self.conversation_interface.stop()
        return 'ok'


def get_suspension_tf(self, susp_data):
    print("My TASKER -- get_suspension_tf")
    print("My TASKER")
    print("My TASKER")
    rclpy.logging.get_logger(self.__class__.__name__).info(
        f'Executing state: {self.__class__.__name__}')
    print('GetSuspend.execute')
    print(f"susp data: {susp_data.getData()}")
    print(f"susp data[0]: {susp_data.getData()[0]}")
    data = susp_data.getData()
    fsm_executable = None
    for idx in range(2, len(data), 2):
        if data[idx] == 'executable':
            fsm_executable = data[idx+1]
        elif data[idx] == 'rosrun':
            ros_pkg = data[idx+1]
            ros_exec = data[idx+2]
            fsm_executable = f"rosrun {ros_pkg} {ros_exec}"
    if fsm_executable is None:
        print("harmoniser did not specify an executable for suspension behaviour, terminating task")
        fsm_executable = "terminate task"

    return fsm_executable


def exe_suspension_tf(self, fsm_es_in):
    print("My TASKER -- exe_suspension_tf")
    print("My TASKER")
    print("My TASKER")
    rclpy.logging.get_logger(self.__class__.__name__).info(
        f'Executing state: {self.__class__.__name__}')
    print('ExecSuspension.execute')
    print(fsm_es_in)
    if fsm_es_in == "terminate task":
        pass
        # return 'shutdown'
    else:
        p = subprocess.Popen(fsm_es_in, shell=True,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        p.wait()
    return "FINISHED"


def wait_tf(self):
    pass


def update_task_tf(self):
    global USE_SMACH_INRTOSPECTION_SERVER
    print("My TASKER -- update_task_tf")
    print("My TASKER")
    print("My TASKER")

    if USE_SMACH_INRTOSPECTION_SERVER:
        self.sis.stop()
        self.sis.clear()

    print("SWAPPING")
    self.swap_state('ExecFSM', MainState())

    if USE_SMACH_INRTOSPECTION_SERVER:
        self.sis.start()
    pass


def initialise_tf(self):
    print("My TASKER -- initialise")
    # self.conversation_interface.start()


def main():
    # da_name = None
    # da_type = None
    # da_id = None
    # da_state_name = []
    # for idx in range(1, len(sys.argv), 2):
    #     if sys.argv[idx] == 'da_name':
    #         da_name = sys.argv[idx+1]
    #     if sys.argv[idx] == 'da_type':
    #         da_type = sys.argv[idx+1]
    #     if sys.argv[idx] == 'da_id':
    #         da_id = sys.argv[idx+1]
    # if da_name == None or da_type == None or da_id == None:
    #     print(
    #         "DA: one of the parameters (<da_name>, <da_type>, or <da_id>) is not specified")
    #     return 1

    da_name = "test_name"
    da_type = "test_type"
    da_id = "1"

    da = DynAgent(da_name, da_id, da_type, ptf_csp, da_state_name)
    print("RUN BRING")
    da.run(MyTaskER(da_state_name, da_name))
    print("BG ENDED")
    return 0


if __name__ == '__main__':
    main()
    print("ALLLLLLL CLOOOOOSSSSEEEEED")
