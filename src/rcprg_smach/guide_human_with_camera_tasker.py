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
import tiago_msgs.msg
import std_msgs

from task_database.srv import GetParamsForScenario, AddParamsForScenario, CloneScenarioWithNewIntent
from language_processor.srv import InitiateConvBasedOnCtx
from rico_context.srv import GetContext, ResetContext, ResetContextResponse, ResetContextRequest
from rico_context.msg import HistoryEvent
from rico_human_detection.msg import coordinates, results

import navigation
from TaskER.TaskER import TaskER
from rcprg_smach import smach_rcprg
from pl_nouns.dictionary_client import DisctionaryServiceClient
import task_manager
import os
from task_database.srv import GetTaskDescription
from pal_common_msgs.msg import DisableAction, DisableActionGoal, DisableGoal
from control_msgs.msg import PointHeadAction, PointHeadActionGoal, PointHeadGoal



ACK_WAIT_MAX_TIME_S = 30

pub_context = rospy.Publisher('/context/push', HistoryEvent, queue_size=10)

class MoveToWithCheckingForHuman(navigation.MoveTo):
    def __init__(self, sim_mode, conversation_interface):
        self.sim_mode = sim_mode
        navigation.MoveTo.__init__(self, sim_mode, conversation_interface, outcomes = ['ok', 'preemption', 'error', 'stall', 'shutdown', 'alarm'])
        self.navigation_max_time_s = 100


    # def move_base_done_cb(self, status, result):
    #     self.is_goal_achieved = True
    #     if self.hazard_trigger == True:
    #         self.move_base_status = status
    #     self.move_base_status = status

    def transition_function(self, userdata):
        global HUMAN_POSE_UPDATE_IN_APPROACH
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        place_name = userdata.move_goal.parameters['place_name']

        assert isinstance(place_name, unicode)
        # answer_id = self.conversation_interface.setAutomaticAnswer( 'q_current_task', u'niekorzystne warunki pogodowe I\'m going to the ' + place_name )
        answer_id = self.conversation_interface.setAutomaticAnswer( 'q_current_task', u'I\'m going to the ' + place_name )

        self.set_destination_pose(userdata)
        pose = userdata.move_goal.parameters['pose']
        place_name = userdata.move_goal.parameters['place_name']

        print "POSE: ", pose
        print "PLACE_NAME: ", place_name

        if self.sim_mode == 'sim':
            for i in range(50):
                if self.is_suspension_flag() != None:
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    self.request_preempt()
                    return 'preemption'

                rospy.sleep(0.2)
            self.conversation_interface.removeAutomaticAnswer(answer_id)
            return 'ok'
        else:
            goal = MoveBaseGoal()
            goal.target_pose.pose = pose
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()

            # turn off auto head motion
            if self.sim_mode not in ['sim', 'gazebo']:
                client_autonomous_head = actionlib.SimpleActionClient('/pal_head_manager/disable', DisableAction)
                client_autonomous_head.wait_for_server()
                client_autonomous_head.send_goal(DisableGoal())
                # client_autonomous_head.wait_for_result()
            # move head to detect objects on the floor
            client_move_head = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
            client_move_head.wait_for_server()
            point_head_goal = PointHeadGoal()
            point_head_goal.target.header.frame_id = 'base_link'
            point_head_goal.target.point.x = 1.5
            point_head_goal.pointing_axis.z = 1
            point_head_goal.pointing_frame = 'xtion_rgb_optical_frame'
            point_head_goal.min_duration.secs = 1
            point_head_goal.max_velocity = 1
            client_move_head.send_goal(point_head_goal)
            # client_move_head.wait_for_result()
            # start moving
            client.send_goal(goal, self.move_base_done_cb, self.move_base_active_cb, self.move_base_feedback_cb)

            # action_feedback = GoActionFeedback()
            # action_result = GoActionResult()
            # action_result.result.is_goal_accomplished = False
            # userdata.nav_result = action_result.result

            start_time = rospy.Time.now()
            last_human_update = rospy.Time.now()
            self.is_goal_achieved = False
            while ( (self.is_goal_achieved == False or self.move_base_status == GoalStatus.PREEMPTED)):
                # action_feedback.feedback.current_pose = self.current_pose

                # userdata.nav_feedback = action_feedback.feedback
                # userdata.nav_actual_pose = self.current_pose

                end_time = rospy.Time.now()
                loop_time = end_time - start_time
                loop_time_s = loop_time.secs

                if self.__shutdown__:
                    client.cancel_all_goals()
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    self.service_preempt()
                    return 'shutdown'

                if loop_time_s > self.navigation_max_time_s:
                    # break the loop, end with error state
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    rospy.logwarn('State: Navigation took too much time, returning error')
                    client.cancel_all_goals()
                    return 'stall'

                #in case losing human from camera
                msg_results = rospy.wait_for_message("/results", results)
                print(msg_results)
                if msg_results.is_human_detected == "NO":
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    client.cancel_all_goals()
                    print("GOAL HAS BEEN CANCELED")
                    return 'alarm'
                elif float(msg_results.distance) > 1.5:
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    client.cancel_all_goals()
                    print("GOAL HAS BEEN CANCELED")
                    return 'alarm'

                if self.update_destination_pose(userdata):
                    goal.target_pose.pose = userdata.move_goal.parameters['pose']
                    client.send_goal(goal, self.move_base_done_cb, self.move_base_active_cb, self.move_base_feedback_cb)

                if self.is_suspension_flag() != None:
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    client.cancel_all_goals()
                    self.service_preempt()
                    return 'preemption'

                rospy.sleep(0.1)

            # Manage state of the move_base action server
            self.conversation_interface.removeAutomaticAnswer(answer_id)
            # move head ahead
            client_move_head.wait_for_server()
            point_head_goal = PointHeadGoal()
            print  "point_head_goal:\n", point_head_goal
            point_head_goal.target.header.frame_id = 'torso_lift_link'
            point_head_goal.target.point.x = 1
            point_head_goal.target.point.z = 0.18
            point_head_goal.pointing_axis.z = 1
            point_head_goal.pointing_frame = 'xtion_rgb_optical_frame'
            point_head_goal.min_duration.secs = 1
            point_head_goal.max_velocity = 1
            client_move_head.send_goal(point_head_goal)

            # turn on auto head motion

            if self.sim_mode not in ['sim', 'gazebo']:
                client_autonomous_head = actionlib.SimpleActionClient('/pal_head_manager/disable', DisableAction)
                client_autonomous_head.cancel_all_goals()

            # Here check move_base DONE status
            if self.move_base_status == GoalStatus.PENDING:
                # The goal has yet to be processed by the action server
                raise Exception('Wrong move_base action status: "PENDING"')
            elif self.move_base_status == GoalStatus.ACTIVE:
                # The goal is currently being processed by the action server
                raise Exception('Wrong move_base action status: "ACTIVE"')
            elif self.move_base_status == GoalStatus.PREEMPTED:
                # The goal received a cancel request after it started executing
                #   and has since completed its execution (Terminal State)
                return 'preemption'
            elif self.move_base_status == GoalStatus.SUCCEEDED:
                # The goal was achieved successfully by the action server (Terminal State)
                return 'ok'
            elif self.move_base_status == GoalStatus.ABORTED:
                # The goal was aborted during execution by the action server due
                #    to some failure (Terminal State)
                return 'stall'
            elif self.move_base_status == GoalStatus.REJECTED:
                # The goal was rejected by the action server without being processed,
                #    because the goal was unattainable or invalid (Terminal State)
                return 'error'
            elif self.move_base_status == GoalStatus.PREEMPTING:
                # The goal received a cancel request after it started executing
                #    and has not yet completed execution
                raise Exception('Wrong move_base action status: "PREEMPTING"')
            elif self.move_base_status == GoalStatus.RECALLING:
                # The goal received a cancel request before it started executing,
                #    but the action server has not yet confirmed that the goal is canceled
                raise Exception('Wrong move_base action status: "RECALLING"')
            elif self.move_base_status == GoalStatus.RECALLED:
                # The goal received a cancel request before it started executing
                #    and was successfully cancelled (Terminal State)
                return 'preemption'
            elif self.move_base_status == GoalStatus.LOST:
                # An action client can determine that a goal is LOST. This should not be
                #    sent over the wire by an action server
                raise Exception('Wrong move_base action status: "LOST"')
            else:
                raise Exception('Wrong move_base action status value: "' + str(self.move_base_status) + '"')

class WaitForHuman(TaskER.BlockingState):
    def __init__(self, sim_mode):
        assert sim_mode in ['sim', 'gazebo', 'real']

        TaskER.BlockingState.__init__(self,tf_freq=10,
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])


    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        while(True):
            msg_results = rospy.wait_for_message("/results", results)
            print(".", msg_results.distance, ".")
            # if(msg_results.is_human_detected == "YES"):
            #     return 'ok'
            if msg_results.is_human_detected == "YES":
                if(float(msg_results.distance) < 1.5):
                    return 'ok'

            if self.preempt_requested():
                self.service_preempt()
                return 'preemption'

            if self.__shutdown__:
                return 'shutdown'


class MoveToComplexWithHumanTracking(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.StateMachine.__init__(self, outcomes=['FINISHED', 'PREEMPTED', 'FAILED', 'shutdown'],
                                            input_keys=['goal', 'susp_data'])

        self.description = u'I\'m going to the particular place'

        with self:
            smach_rcprg.StateMachine.add('RememberCurrentPose', navigation.RememberCurrentPose(sim_mode),
                                    transitions={'ok':'UnderstandGoal', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'current_pose':'current_pose'})

            smach_rcprg.StateMachine.add('UnderstandGoal', navigation.UnderstandGoal(sim_mode, conversation_interface, kb_places),
                                    transitions={'ok':'SayImGoingTo', 'preemption':'PREEMPTED', 'error': 'SayIdontKnow',
                                    'shutdown':'shutdown'},
                                    remapping={'in_current_pose':'current_pose', 'goal_pose':'goal', 'move_goal':'move_goal'})

            smach_rcprg.StateMachine.add('SayImGoingTo', navigation.SayImGoingTo(sim_mode, conversation_interface),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal'})

            smach_rcprg.StateMachine.add('MoveTo', MoveToWithCheckingForHuman(sim_mode, conversation_interface),
                                    transitions={'ok':'SayIArrivedTo', 'preemption':'PREEMPTED', 'error': 'FAILED', 'stall':'ClearCostMaps',
                                    'shutdown':'shutdown', 'alarm':'WaitForHuman'},
                                    remapping={'move_goal':'move_goal', 'susp_data':'susp_data'})
            smach_rcprg.StateMachine.add('WaitForHuman', WaitForHuman(sim_mode),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'})

            smach_rcprg.StateMachine.add('ClearCostMaps', navigation.ClearCostMaps(sim_mode),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'})

            smach_rcprg.StateMachine.add('SayIArrivedTo', navigation.SayIArrivedTo(sim_mode, conversation_interface),
                                    transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal'})

            smach_rcprg.StateMachine.add('SayIdontKnow', navigation.SayIdontKnow(sim_mode, conversation_interface),
                                    transitions={'ok':'FAILED', 'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal'})


class SayIFinished(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        TaskER.BlockingState.__init__(self, input_keys=[], output_keys=[],
                                      outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.reset_context = rospy.ServiceProxy('/context/reset', ResetContext)

        self.description = u'Mówię, że zakończyłem'

    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(
            rospy.get_name(), self.__class__.__name__))

        self.conversation_interface.speakNowBlocking(
            u'I finished performing the task')

        self.reset_context()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'


class GuideHumanWithCamera(smach_rcprg.StateMachine):

    def __init__(self, sim_mode, conversation_interface, kb_places, task_parameters):
        input_keys = []

        for idx in range(0, len(task_parameters), 2):
            param_name = task_parameters[idx]
            param_value = task_parameters[idx+1]
            
            input_keys.append(param_name)

        input_keys.extend(['susp_data', 'goal']);

        smach_rcprg.StateMachine.__init__(self, input_keys=input_keys, output_keys=['susp_data'],
                                          outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])

        for idx in range(0, len(task_parameters), 2):
            param_name = task_parameters[idx]
            param_value = task_parameters[idx+1]

            print('Parameters setting to userdate: ', param_name, param_value)

            setattr(self.userdata, param_name, param_value)

        self.userdata.max_lin_vel = 0.4
        self.userdata.max_lin_accel = 0.5

        self.userdata.default_height = 0.2
        self.userdata.lowest_height = 0.0

        self.userdata.goal = task_manager.PoseDescription({'place_name': self.userdata.place})

        self.description = u'Going to a place'

        with self:
            smach_rcprg.StateMachine.add('MoveToPlace', MoveToComplexWithHumanTracking(sim_mode, conversation_interface, kb_places),
                                         transitions={'FINISHED': 'SayIFinished', 'PREEMPTED': 'PREEMPTED', 'FAILED': 'FAILED',
                                                      'shutdown': 'shutdown'},
                                         remapping={'susp_data': 'susp_data'})

            smach_rcprg.StateMachine.add('SayIFinished', SayIFinished(sim_mode, conversation_interface),
                                         transitions={'ok': 'FINISHED', 'shutdown': 'shutdown', 'preemption': 'PREEMPTED', 'error': 'FAILED'})
