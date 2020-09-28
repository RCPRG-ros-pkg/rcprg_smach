#!/usr/bin/env python
# encoding: utf8

import rospy
import smach
import smach_ros
import dynamic_reconfigure.client
import actionlib
import math
import threading
import copy

from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import std_srvs.srv as std_srvs

from tf.transformations import quaternion_from_euler, euler_from_quaternion

import smach_rcprg
from task_manager import PoseDescription

NAVIGATION_MAX_TIME_S = 100


def makePose(x, y, theta):
    q = quaternion_from_euler(theta, 0, 0)
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.orientation.w = q[0]
    result.orientation.x = q[1]
    result.orientation.y = q[2]
    result.orientation.z = q[3]
    return result

class RememberCurrentPose(smach_rcprg.State):
    def __init__(self, sim_mode):
        smach_rcprg.State.__init__(self, output_keys=['current_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        assert sim_mode in ['sim', 'gazebo', 'real']
        self.sim_mode = sim_mode
        if self.sim_mode in ['gazebo', 'real']:
            self.__lock__ = threading.Lock()
            self.current_pose = None
            self.sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback)

        self.description = u'Zapamiętuję bieżącą pozycję'

    def callback(self, data):
        self.__lock__.acquire()
        self.current_pose = copy.copy(data)
        self.__lock__.release()

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        if self.sim_mode == 'sim':
            userdata.current_pose = PoseDescription( {'pose':makePose(0, 0, 0)} )
            if self.preempt_requested():
                self.service_preempt()
                return 'preemption'

            if self.__shutdown__:
                return 'shutdown'
            return 'ok'
        else:
            pose_valid = False
            for i in range(10):
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preemption'

                self.__lock__.acquire()
                if not self.current_pose is None:
                    pose_valid = True
                    userdata.current_pose = PoseDescription( {'pose':self.current_pose.pose.pose} )
                self.__lock__.release()

                if self.__shutdown__:
                    return 'shutdown'
                if pose_valid:
                    return 'ok'
                rospy.sleep(0.1)
            if self.__shutdown__:
                return 'shutdown'
            return 'error'

class UnderstandGoal(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.State.__init__(self, input_keys=['in_current_pose', 'goal_pose'], output_keys=['move_goal'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        assert sim_mode in ['sim', 'gazebo', 'real']
        self.sim_mode = sim_mode
        self.kb_places = kb_places

        self.description = u'Próbuję zrozumieć zadany cel'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        #assert isinstance( userdata.goal_pose, PoseDescription )

        print userdata.goal_pose
        if 'place_name' in userdata.goal_pose.parameters:
            place_name = userdata.goal_pose.parameters['place_name']
            pose_valid = False
            place_name_valid = True
            pose = None
            print 'place_name', place_name
        elif 'pose' in userdata.goal_pose.parameters:
            pose = userdata.goal_pose.parameters['pose']
            pose_valid = True
            place_name_valid = False
            place_name = u'nieznane'
            print 'pose', pose
        else:
            raise Exception('Parameters are missing')
        #elif not userdata.nav_goal_pose is None:
        #    pose = userdata.nav_goal_pose.pose
        #    pose_valid = userdata.nav_goal_pose.pose_valid
        #    place_name_valid = userdata.nav_goal_pose.place_name_valid
        #    if place_name_valid:
        #        if isinstance(userdata.nav_goal_pose.place_name, str):
        #            place_name = userdata.nav_goal_pose.place_name.decode('utf-8')
        #        elif isinstance(userdata.nav_goal_pose.place_name, unicode):
        #            place_name = userdata.nav_goal_pose.place_name
        #        else:
        #            raise Exception('Unexpected type of place_name: "' + str(type(place_name)) + '"')
        #    else:
        #        place_name = u'nieznane'

        assert isinstance(place_name, unicode)

        if self.sim_mode == 'sim':
            pose = makePose(0, 0, 0)
        else:
            if self.sim_mode == 'real':
                mc_name = 'real'
            elif self.sim_mode == 'gazebo':
                mc_name = 'sim'

            if not pose_valid:
                if not place_name_valid:
                    print 'UnderstandGoal place_name is not valid'
                    return 'error'

                current_pose = userdata.in_current_pose
                pt_start = (current_pose.parameters['pose'].position.x, current_pose.parameters['pose'].position.y)

                try:
                    pl = self.kb_places.getPlaceByName(place_name, mc_name)
                except:
                    userdata.move_goal = PoseDescription( {'pose':pose, 'place_name':place_name} )
                    print 'UnderstandGoal place_name is not valid'
                    return 'error'

                if pl.getType() == 'point':
                    pt_dest = pl.getPt()
                    norm = pl.getN()
                    angle_dest = -math.atan2(norm[1], norm[0])
                    pt = pt_dest
                    pt_dest = (pt_dest[0]+norm[0], pt_dest[1]+norm[1])
                    print 'UnderstandGoal place type: point'
                    print 'pt: {}, pt_dest: {}, norm: {}, angle_dest: {}'.format(pt, pt_dest, norm, angle_dest)
                elif pl.getType() == 'volumetric':
                    pt_dest = self.kb_places.getClosestPointOfPlace(pt_start, pl.getId(), mc_name, dbg_output_path = '/home/dseredyn/tiago_public_ws/img')
                    angle_dest = 0.0
                else:
                    raise Exception('Unknown place type: "' + pl.getType() + '"')
                pose = makePose(pt_dest[0], pt_dest[1], angle_dest)
            else:
                place_pos = (pose.position.x, pose.position.y)
                result_place_id = self.kb_places.whatIsAt(place_pos, mc_name)
                if result_place_id is None:
                    place_name = u'nieznane'
                else:
                    pl = self.kb_places.getPlaceById(result_place_id, mc_name)
                    place_name = pl.getName()   # returns unicode
                    assert isinstance(place_name, unicode)

        assert isinstance(place_name, unicode)

        result = PoseDescription( {'pose':pose, 'place_name':place_name} )

        if self.preempt_requested():
            self.service_preempt()
            return 'preemption'

        if self.__shutdown__:
            return 'shutdown'

        userdata.move_goal = result
        return 'ok'

class SayImGoingTo(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.State.__init__(self, input_keys=['move_goal'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Mówię dokąd jadę'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        pose = userdata.move_goal.parameters['pose']
        place_name = userdata.move_goal.parameters['place_name']

        assert isinstance(place_name, unicode)
        #self.conversation_interface.addSpeakSentence( u'Jadę do {"' + place_name + u'", dopelniacz}' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe jadę do {"' + place_name + u'", dopelniacz}' )

        if self.preempt_requested():
            #self.conversation_interface.removeExpected('q_current_task')
            self.service_preempt()
            return 'preemption'

        if self.__shutdown__:
            return 'shutdown'

        return 'ok'

class SayIdontKnow(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.State.__init__(self, input_keys=['move_goal'],
                             outcomes=['ok', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Mówię, że nie wiem o co chodzi'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        place_name = userdata.move_goal.parameters['place_name']
        assert isinstance(place_name, unicode)
        #self.conversation_interface.addSpeakSentence( u'Nie wiem gdzie jest {"' + place_name + u'", mianownik}' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe nie wiem gdzie jest {"' + place_name + u'", mianownik}' )

        if self.preempt_requested():
            #self.conversation_interface.removeExpected('q_current_task')
            self.service_preempt()
            return 'preemption'

        if self.__shutdown__:
            return 'shutdown'

        return 'ok'

class SayIArrivedTo(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface):
        smach_rcprg.State.__init__(self, input_keys=['move_goal'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Mówię, że dojechałem do celu'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        pose = userdata.move_goal.parameters['pose']
        place_name = userdata.move_goal.parameters['place_name']
        assert isinstance(place_name, unicode)
        #self.conversation_interface.addSpeakSentence( u'Dojechałem do {"' + place_name + u'", dopelniacz}' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe dojechałem do {"' + place_name + u'", dopelniacz}' )

        if self.preempt_requested():
            self.service_preempt()
            return 'preemption'

        if self.__shutdown__:
            return 'shutdown'
        #self.conversation_interface.addSpeakSentence( 'Dojechałem do pozycji ' + str(pose.position.x) + ', ' + str(pose.position.y) )
        return 'ok'

class SetNavParams(smach_rcprg.State):
    def __init__(self, sim_mode):
        assert sim_mode in ['sim', 'gazebo', 'real']
        self.sim_mode = sim_mode
        if sim_mode in ['gazebo', 'real']:
            base_local_planner = rospy.get_param('/move_base/base_local_planner')
            self.local_planner_name = base_local_planner.split('/')[-1]

            print 'SetNavParams: detected local planner: ' + self.local_planner_name

            self.dynparam_client = dynamic_reconfigure.client.Client('/move_base/' + self.local_planner_name)

            # Ensure that we know what we are doing.
            # Check if the given parameters are present in the configuration.
            config = self.dynparam_client.get_configuration()
            if self.local_planner_name == 'PalLocalPlanner':
                assert 'max_vel_x' in config
                assert 'acc_lim_x' in config
            elif self.local_planner_name == 'EBandPlannerROS':
                assert 'max_vel_lin' in config
                assert 'max_acceleration' in config
            elif self.local_planner_name == 'TebLocalPlannerROS':
                assert 'max_vel_x' in config
                assert 'acc_lim_x' in config
            else:
                raise Exception('Local planner "' + self.local_planner_name + '" is not supported.')

        smach_rcprg.State.__init__(self, input_keys=['max_lin_vel_in', 'max_lin_accel_in'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.description = u'Zmieniam parametry ruchu'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        if self.sim_mode == 'sim':
            rospy.sleep(1.0)
        else:
            max_lin_vel = userdata.max_lin_vel_in
            max_lin_accel = userdata.max_lin_accel_in

            # setting planner params so it the robot moves slowly
            # params = userdata.set_nav_params_params
            # Differrent planners have different parameters
            if self.local_planner_name == 'PalLocalPlanner':
                params = {
                    'max_vel_x': max_lin_vel,
                    'acc_lim_x': max_lin_accel
                }
            elif self.local_planner_name == 'EBandPlannerROS':
                params = {
                    'max_vel_lin': max_lin_vel,
                    'max_acceleration': max_lin_accel
                }
            elif self.local_planner_name == 'TebLocalPlannerROS':
                params = {
                    'max_vel_x': max_lin_vel,
                    'acc_lim_x': max_lin_accel
                }
            else:
                raise Exception('Local planner "' + self.local_planner_name + '" is not supported.')

            config = self.dynparam_client.update_configuration(params)

        if self.preempt_requested():
            self.service_preempt()
            return 'preemption'

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class MoveTo(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface):
        assert sim_mode in ['sim', 'gazebo', 'real']
        self.current_pose = Pose()
        self.is_feedback_received = False
        self.move_base_status = GoalStatus.PENDING
        self.is_goal_achieved = False
        self.sim_mode = sim_mode
        self.conversation_interface = conversation_interface

        smach_rcprg.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error', 'stall', 'shutdown'],
                             input_keys=['move_goal'])

        self.description = u'Jadę'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        place_name = userdata.move_goal.parameters['place_name']

        assert isinstance(place_name, unicode)
        answer_id = self.conversation_interface.setAutomaticAnswer( 'q_current_task', u'niekorzystne warunki pogodowe jadę do {"' + place_name + u'", dopelniacz}' )

        pose = userdata.move_goal.parameters['pose']
        place_name = userdata.move_goal.parameters['place_name']

        if self.sim_mode == 'sim':
            for i in range(50):
                if self.preempt_requested():
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    self.service_preempt()
                    return 'preemption'

                rospy.sleep(0.1)
            self.conversation_interface.removeAutomaticAnswer(answer_id)
            return 'ok'
        else:
            goal = MoveBaseGoal()
            goal.target_pose.pose = pose
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()

            # start moving
            client.send_goal(goal, self.move_base_done_cb, self.move_base_active_cb, self.move_base_feedback_cb)

            # action_feedback = GoActionFeedback()
            # action_result = GoActionResult()
            # action_result.result.is_goal_accomplished = False
            # userdata.nav_result = action_result.result

            start_time = rospy.Time.now()

            self.is_goal_achieved = False
            while self.is_goal_achieved == False:
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

                if loop_time_s > NAVIGATION_MAX_TIME_S:
                    # break the loop, end with error state
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    rospy.logwarn('State: Navigation took too much time, returning error')
                    client.cancel_all_goals()
                    return 'stall'

                if self.preempt_requested():
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    client.cancel_all_goals()
                    self.service_preempt()
                    return 'preemption'

                rospy.sleep(0.1)

            # Manage state of the move_base action server
            self.conversation_interface.removeAutomaticAnswer(answer_id)

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

    def move_base_feedback_cb(self, feedback):
        self.current_pose = feedback.base_position.pose
        self.is_feedback_received = True

    def move_base_done_cb(self, status, result):
        self.is_goal_achieved = True
        self.move_base_status = status

    def move_base_active_cb(self):
        # Do nothing
        return

class TurnAround(smach_rcprg.State):
    def __init__(self, sim_mode, conversation_interface):
        assert sim_mode in ['sim', 'gazebo', 'real']
        self.current_pose = Pose()
        self.is_feedback_received = False
        self.move_base_status = GoalStatus.PENDING
        self.is_goal_achieved = False
        self.sim_mode = sim_mode
        self.conversation_interface = conversation_interface

        smach_rcprg.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error', 'stall', 'shutdown'],
                             input_keys=['current_pose'])

        self.description = u'Odwracam się'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        pose = userdata.current_pose.parameters['pose']

        theta, beta, alpha = euler_from_quaternion( (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z) )
        print 'TurnAround current theta: {} {} {}'.format(alpha, beta, theta)
        if theta < 0:
            new_pose = makePose(pose.position.x, pose.position.y, theta+math.pi)
        else:
            new_pose = makePose(pose.position.x, pose.position.y, theta-math.pi)

        answer_id = self.conversation_interface.setAutomaticAnswer( 'q_current_task', u'niekorzystne warunki pogodowe odwracam się' )

        if self.sim_mode == 'sim':
            for i in range(50):
                if self.preempt_requested():
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    self.service_preempt()
                    return 'preemption'

                rospy.sleep(0.1)
            self.conversation_interface.removeAutomaticAnswer(answer_id)
            return 'ok'
        else:
            goal = MoveBaseGoal()
            goal.target_pose.pose = new_pose
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()

            # start moving
            client.send_goal(goal, self.move_base_done_cb, self.move_base_active_cb, self.move_base_feedback_cb)

            # action_feedback = GoActionFeedback()
            # action_result = GoActionResult()
            # action_result.result.is_goal_accomplished = False
            # userdata.nav_result = action_result.result

            start_time = rospy.Time.now()

            self.is_goal_achieved = False
            while self.is_goal_achieved == False:
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

                if loop_time_s > NAVIGATION_MAX_TIME_S:
                    # break the loop, end with error state
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    rospy.logwarn('State: Navigation took too much time, returning error')
                    client.cancel_all_goals()
                    return 'stall'

                if self.preempt_requested():
                    self.conversation_interface.removeAutomaticAnswer(answer_id)
                    client.cancel_all_goals()
                    self.service_preempt()
                    return 'preemption'

                rospy.sleep(0.1)

            # Manage state of the move_base action server
            self.conversation_interface.removeAutomaticAnswer(answer_id)

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

    def move_base_feedback_cb(self, feedback):
        self.current_pose = feedback.base_position.pose
        self.is_feedback_received = True

    def move_base_done_cb(self, status, result):
        self.is_goal_achieved = True
        self.move_base_status = status

    def move_base_active_cb(self):
        # Do nothing
        return

class ClearCostMaps(smach_rcprg.State):
    def __init__(self, sim_mode):
        assert sim_mode in ['sim', 'gazebo', 'real']
        if sim_mode == 'sim':
            self.clear_costmaps = None
        else:
            rospy.wait_for_service('/move_base/clear_costmaps')
            #try:
            self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.Empty)
            #except rospy.ServiceException, e:
            #    print "Service call failed: %s"%e
            #    self.clear_costmaps = None

        smach_rcprg.State.__init__(self,
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.description = u'Czyszczę mapę kosztów'

    def execute(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))

        if not self.clear_costmaps is None:
            self.clear_costmaps()
        rospy.sleep(0.5)

        if self.preempt_requested():
            self.service_preempt()
            return 'preemption'

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class MoveToComplex(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.StateMachine.__init__(self, outcomes=['FINISHED', 'PREEMPTED', 'FAILED', 'shutdown'],
                                            input_keys=['goal'])

        self.description = u'Jadę do określonego miejsca'

        with self:
            smach_rcprg.StateMachine.add('RememberCurrentPose', RememberCurrentPose(sim_mode),
                                    transitions={'ok':'UnderstandGoal', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'current_pose':'current_pose'})

            smach_rcprg.StateMachine.add('UnderstandGoal', UnderstandGoal(sim_mode, conversation_interface, kb_places),
                                    transitions={'ok':'SayImGoingTo', 'preemption':'PREEMPTED', 'error': 'SayIdontKnow',
                                    'shutdown':'shutdown'},
                                    remapping={'in_current_pose':'current_pose', 'goal_pose':'goal', 'move_goal':'move_goal'})

            smach_rcprg.StateMachine.add('SayImGoingTo', SayImGoingTo(sim_mode, conversation_interface),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal'})

            smach_rcprg.StateMachine.add('MoveTo', MoveTo(sim_mode, conversation_interface),
                                    transitions={'ok':'SayIArrivedTo', 'preemption':'PREEMPTED', 'error': 'FAILED', 'stall':'ClearCostMaps',
                                    'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal'})

            smach_rcprg.StateMachine.add('ClearCostMaps', ClearCostMaps(sim_mode),
                                    transitions={'ok':'MoveTo', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'})

            smach_rcprg.StateMachine.add('SayIArrivedTo', SayIArrivedTo(sim_mode, conversation_interface),
                                    transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal'})

            smach_rcprg.StateMachine.add('SayIdontKnow', SayIdontKnow(sim_mode, conversation_interface),
                                    transitions={'ok':'FAILED', 'shutdown':'shutdown'},
                                    remapping={'move_goal':'move_goal'})

#    def execute(self, userdata):
#        if not 'place_name' in userdata.goal.parameters or userdata.goal.parameters['place_name'] is None:
#            self.description = u'Gdzieś jadę'
#        else:
#            place_name = userdata.goal.parameters['place_name']
#            self.description = u'Jadę do {"' + place_name + u'", dopelniacz}'
#        return super(MoveToComplex, self).execute(userdata)


#    def execute(self, userdata):
#        if not 'place_name' in userdata.goal.parameters or userdata.goal.parameters['place_name'] is None:
#            self.description = u'Gdzieś jadę'
#        else:
#            place_name = userdata.goal.parameters['place_name']
#            self.description = u'Jadę do {"' + place_name + u'", dopelniacz}'
#        return super(MoveToComplexTorsoMid, self).execute(userdata)
