#!/usr/bin/env python2
# -*- coding: utf-8 -*- 

import rospy
import math
import PyKDL
from threading import Thread

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
import tf_conversions.posemath as pm
from rcprg_ros_utils import exitError, MarkerPublisher
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3

from TaskER.TaskER import TaskER
from rcprg_smach import smach_rcprg
import navigation
import manipulation


class MarkerPublisherThread:
    def threaded_function(self, obj):
        pub = MarkerPublisher("attached_objects")
        while not self.stop_thread:
            pub.publishSinglePointMarker(PyKDL.Vector(), 1, r=1, g=0, b=0, a=1, namespace='default', frame_id=obj.link_name, m_type=Marker.CYLINDER, scale=Vector3(0.02, 0.02, 1.0), T=pm.fromMsg(obj.object.primitive_poses[0]))
            try:
                rospy.sleep(0.1)
            except:
                break

        try:
            pub.eraseMarkers(0, 10, namespace='default')
            rospy.sleep(0.5)
        except:
            pass

    def __init__(self, obj):
        self.thread = Thread(target = self.threaded_function, args = (obj, ))

    def start(self):
        self.stop_thread = False
        self.thread.start()

    def stop(self):
        self.stop_thread = True
        self.thread.join()

class SetBaseDestination(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface):
        TaskER.BlockingState.__init__(self, input_keys=['dest_name'], output_keys=['goal_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])
        self.conversation_interface = conversation_interface

        self.description = u'Znajduje cel ruchu'
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Ustalam gdzie jest miejsce przedmiotu' )
        if isinstance(userdata.dest_name, str):
            dest_name = userdata.dest_name.decode('utf-8')
        dest_name = userdata.dest_name.encode('utf-8').decode('utf-8')
        userdata.goal_pose = navigation.PoseDescription({'place_name':unicode(dest_name)})

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class BringJarSuspension(smach_rcprg.StateMachine):
    def __init__(self, sim_mode, conversation_interface, kb_places):
        smach_rcprg.StateMachine.__init__(self, input_keys=['object_container','bring_destination','end_pose','susp_data'], output_keys=['susp_data'],
                                        outcomes=['PREEMPTED',
                                                    'FAILED',
                                                    'FINISHED', 'shutdown'])
        self.userdata.max_lin_vel = 0.2
        self.userdata.max_lin_accel = 0.5
        # TODO: use knowledge base for this:
        self.userdata.default_height = 0.2
        self.userdata.lowest_height = 0.0

        self.description = u'Podaję rzecz'
        worker = manipulation.VelmaTaskExecutor()
        object1 = AttachedCollisionObject()
        object1.link_name = "left_HandGripLink"
        object1.object.header.frame_id = "left_HandGripLink"
        object1.object.id = "object1"
        object1_prim = SolidPrimitive()
        object1_prim.type = SolidPrimitive.CYLINDER
        object1_prim.dimensions=[None, None]    # set initial size of the list to 2
        object1_prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT] = 0.25
        object1_prim.dimensions[SolidPrimitive.CYLINDER_RADIUS] = 0.06
        object1_pose = pm.toMsg(PyKDL.Frame(PyKDL.Rotation.RotY(math.pi/2)))
        object1.object.primitives.append(object1_prim)
        object1.object.primitive_poses.append(object1_pose)
        object1.object.operation = CollisionObject.ADD
        object1.touch_links = [
            'right_FtSensorLink',
            'right_HandGripLink',
            'right_gripper_mount_link',
            'right_HandPalmLink',
            'right_HandFingerOneKnuckleOneLink',
            'right_HandFingerOneKnuckleTwoLink',
            'right_HandFingerOneKnuckleThreeLink',
            'right_HandFingerTwoKnuckleOneLink',
            'right_HandFingerTwoKnuckleTwoLink',
            'right_HandFingerTwoKnuckleThreeLink',
            'right_HandFingerThreeKnuckleOneLink',
            'right_HandFingerThreeKnuckleTwoLink',
            'right_HandFingerThreeKnuckleThreeLink']
        marker_publisher = MarkerPublisherThread(object1)


        self.userdata.take_out_pose = navigation.PoseDescription({'place_name':unicode('take_out_pose')})
        self.userdata.help_to_open = navigation.PoseDescription({'place_name':unicode('help_to_open')})

        with self:

            smach_rcprg.StateMachine.add('SetContainerDestination', SetBaseDestination(sim_mode, conversation_interface),
                                    transitions={'ok':'SetNavParams', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'dest_name':'object_container'})

            smach_rcprg.StateMachine.add('SetNavParams', navigation.SetNavParams(sim_mode),
                                    transitions={'ok':'PrepareToMoveBase', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'max_lin_vel_in':'max_lin_vel', 'max_lin_accel_in':'max_lin_accel'})

            smach_rcprg.StateMachine.add('PrepareToMoveBase', manipulation.PrepareToMoveBase(sim_mode, conversation_interface, worker),
                                    transitions={'ok':'HideHands', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('HideHands', manipulation.HideHands(sim_mode, conversation_interface, worker),
                                    transitions={'ok':'MoveToCabinet', 'preemption':'PREEMPTED', 'error': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={})

            smach_rcprg.StateMachine.add('MoveToCabinet', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
                                    transitions={'FINISHED':'EmergencyPutDownObject', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
                                    'shutdown':'shutdown'},
                                    remapping={'goal':'goal_pose', 'susp_data':'susp_data'})

            # smach_rcprg.StateMachine.add('PrepareToGrip', manipulation.PrepareToGrip(sim_mode, conversation_interface, worker),
            #                         transitions={'ok':'OpenDoor', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={})

            # smach_rcprg.StateMachine.add('OpenDoor', manipulation.OpenDoor(sim_mode, conversation_interface, worker),
            #                         transitions={'ok':'TakeOutObjectLeft', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'object_container_pose':'object_container_pose'})

            # smach_rcprg.StateMachine.add('PrepareToMoveBase2', manipulation.PrepareToMoveBase(sim_mode, conversation_interface, worker),
            #                         transitions={'ok':'CorrectBasePose2', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={})

            # smach_rcprg.StateMachine.add('CorrectBasePose2', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
            #                         transitions={'FINISHED':'LookForObject', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'goal':'take_out_pose', 'susp_data':'susp_data'})

            # smach_rcprg.StateMachine.add('LookForObject', manipulation.LookForObject(sim_mode, conversation_interface, worker),
            #                         transitions={'ok':'ApproachObject', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={})

            # smach_rcprg.StateMachine.add('ApproachObject', manipulation.ApproachObject(sim_mode, conversation_interface, worker),
            #                         transitions={'ok':'TakeOutObject', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={})

            # smach_rcprg.StateMachine.add('TakeOutObjectLeft', manipulation.TakeOutObjectLeft(sim_mode, conversation_interface, worker, marker_publisher),
            #                         transitions={'ok':'EmergencyPutDownObject', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={})

            smach_rcprg.StateMachine.add('EmergencyPutDownObject', manipulation.EmergencyPutDownObject(sim_mode, conversation_interface, worker, marker_publisher),
                                    transitions={'ok':'FINISHED', 'preemption':'PREEMPTED', 'error':'FAILED'
                                    ,'shutdown':'shutdown', },
                                    remapping={})

            # smach_rcprg.StateMachine.add('SetBringDestination', SetBaseDestination(sim_mode, conversation_interface),
            #                         transitions={'ok':'PrepareToMoveBase3', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'dest_name':'bring_destination'})

            # smach_rcprg.StateMachine.add('PrepareToMoveBase3', manipulation.PrepareToMoveWithObject(sim_mode, conversation_interface, worker),
            #                         transitions={'ok':'MoveToBringDestination', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={})

            # smach_rcprg.StateMachine.add('MoveToBringDestination', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
            #                         transitions={'FINISHED':'PutDownObjectLeft', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'goal':'goal_pose', 'susp_data':'susp_data'})

            # smach_rcprg.StateMachine.add('PutDownObjectLeft', manipulation.PutDownObjectLeft(sim_mode, conversation_interface, worker, marker_publisher),
            #                         transitions={'ok':'SetEndDestination', 'preemption':'PREEMPTED', 'error':'FAILED'
            #                         ,'shutdown':'shutdown', },
            #                         remapping={})

            # smach_rcprg.StateMachine.add('SetEndDestination', SetBaseDestination(sim_mode, conversation_interface),
            #                         transitions={'ok':'MoveToEnd', 'preemption':'PREEMPTED', 'error': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'dest_name':'end_pose'})

            # smach_rcprg.StateMachine.add('MoveToEnd', navigation.MoveToComplex(sim_mode, conversation_interface, kb_places),
            #                         transitions={'FINISHED':'FINISHED', 'PREEMPTED':'PREEMPTED', 'FAILED': 'FAILED',
            #                         'shutdown':'shutdown'},
            #                         remapping={'goal':'goal_pose', 'susp_data':'susp_data'})
