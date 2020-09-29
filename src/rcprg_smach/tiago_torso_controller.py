#!/usr/bin/env python
# encoding: utf8
import copy
import threading
import rospy
import actionlib

import control_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import navigation
import smach_rcprg


class SetHeight(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.State.__init__(self, input_keys=['torso_height'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface
        assert sim_mode in ['sim', 'gazebo', 'real']
        self.sim_mode = sim_mode
        if self.sim_mode in ['gazebo', 'real']:
            self.torso_controller = TiagoTorsoController()

        self.description = u'Zmieniam wysokość'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        if self.sim_mode == 'sim':
            return 'ok'

        current_height = self.torso_controller.get_torso_height()
        if current_height is None:
            rospy.sleep(2)
            if current_height is None:
                return 'error'

        if abs(current_height - userdata.torso_height) > 0.05:
            self.torso_controller.set_torso_height(userdata.torso_height)
            for i in range(30):
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preemption'

                #if self.conversation_interface.consumeExpected('q_current_task'):
                #    #self.conversation_interface.addSpeakSentence( u'Zmieniam wysokość.' )
                #    self.conversation_interface.speakNowBlocking( u'Zmieniam wysokość.' )
                rospy.sleep(0.1)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class MoveToComplexTorsoMid(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.StateMachine.__init__(self, outcomes=['FINISHED', 'PREEMPTED', 'FAILED', 'shutdown'],
                                            input_keys=['goal'])

        self.userdata.default_height = 0.2

        self.description = u'Jadę do określonego miejsca'

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
                                    transitions={'ok':'SetHeightMid', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal'})

            smach_rcprg.StateMachine.add('SetHeightMid', SetHeight(sim_mode, conversation_interface),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'torso_height':'default_height'})

            smach_rcprg.StateMachine.add('MoveTo', navigation.MoveTo(sim_mode, conversation_interface),
                                    transitions={'ok':'SayIArrivedTo', 'preemption':'PREEMPTED', 'error': 'FAILED', 'stall':'ClearCostMaps',
                                    'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal'})

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

class TiagoHeadController:
    def __init__(self,
                 pointing_frame='xtion_optical_frame',
                 pointing_axis=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.0)):

        self.pointing_axis = pointing_axis
        self.pointing_frame = pointing_frame

        # initialize publisher for setting services
        self.client = actionlib.SimpleActionClient('head_controller/point_head_action', control_msgs.msg.PointHeadAction)

        # Wait for server for 10 seconds.
        # Crash, if there is no such action.
        assert self.client.wait_for_server(timeout=rospy.Duration(5))

    def __send_goal(self, pointting_frame, pointing_axis, target_point, min_duration):
        point = geometry_msgs.msg.PointStamped()
        point.header.frame_id = 'base_link'
        point.header.stamp = rospy.Time.now()
        point.point = target_point

        goal = control_msgs.msg.PointHeadGoal()
        goal.pointing_frame = pointting_frame
        goal.pointing_axis = pointing_axis
        goal.min_duration.secs = min_duration
        goal.target = point

        # Callbacks are not handled right now
        self.client.send_goal(goal)

    def tilt_down(self, min_time=2.0):
        target_point = geometry_msgs.msg.Point(1.0, 0.0, 0.1)
        self.__send_goal(self.pointing_frame, self.pointing_axis, target_point, min_time)

    def tilt_up(self, min_time=2.0):
        target_point = geometry_msgs.msg.Point(1.0, 0.0, 1.75)
        self.__send_goal(self.pointing_frame, self.pointing_axis, target_point, min_time)

    def tilt_forward(self, min_time=2.0):
        target_point = geometry_msgs.msg.Point(1.0, 0.0, 1.25)
        self.__send_goal(self.pointing_frame, self.pointing_axis, target_point, min_time)

    def turn_to_point(self, point, min_time=2.0):
        self.__send_goal(self.pointing_frame, self.pointing_axis, point, min_time)

class TiagoTorsoController:
    def __init__(self):
        # initialize publisher for setting services
        self.client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)

        # Wait for server for 10 seconds.
        # Crash, if there is no such action.
        assert self.client.wait_for_server(timeout=rospy.Duration(5))

        self.__lock__ = threading.Lock()
        self.current_height = None
        self.sub = rospy.Subscriber("/torso_controller/state", control_msgs.msg.JointTrajectoryControllerState, self.callback)

    def callback(self, data):
        self.__lock__.acquire()
        self.current_height = copy.copy(data)
        self.__lock__.release()

    def __send_goal(self, height):
        jtp = trajectory_msgs.msg.JointTrajectoryPoint()
        jtp.time_from_start = rospy.Duration(2)
        jtp.positions.append(height)

        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append('torso_lift_joint')
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.points.append(jtp)

        # Callbacks are not handled right now
        self.client.send_goal(goal)

    def set_torso_height(self, height):
        self.__send_goal(height)

    def get_torso_height(self):
        self.__lock__.acquire()
        if self.current_height is None:
            current_height = None
        else:
            current_height = copy.copy(self.current_height.actual.positions[0])
        self.__lock__.release()
        return current_height
