#!/usr/bin/env python2
# -*- coding: utf-8 -*- 

import rospy
import math
import tf2_ros
import PyKDL
import copy
from threading import Thread

from geometry_msgs.msg import Twist, TransformStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError, MarkerPublisher

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import tf_conversions.posemath as pm
from TaskER.TaskER import TaskER
import navigation


class SetpointGenerator():
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def generate(self, destinationFrame, safetyTranslation):
        rospy.sleep(0.5)
        msg = self.tfBuffer.lookup_transform("map", destinationFrame, rospy.Time(0))
        rot = msg.transform.rotation
        setpointRot = PyKDL.Rotation.Quaternion(rot.x, rot.y, rot.z, rot.w)
        trans = msg.transform.translation
        destinationFrame = PyKDL.Frame(setpointRot, PyKDL.Vector(trans.x, trans.y, trans.z))
        setpointVector = destinationFrame * safetyTranslation
        setpointRot.DoRotZ(math.pi)
        result = (setpointVector.x(), setpointVector.y(), setpointRot.GetRPY()[2])
        print(result)
        return result


class VelmaTaskExecutor():
    def __init__(self):
        self.velma = VelmaInterface()
        if not self.velma.waitForInit(timeout_s=10.0):
            raise Exception("Could not initialize VelmaInterface")

        if self.velma.enableMotors() != 0:
            raise Exception("Could not enable motors")

        self.velma.startHomingHP()
        if self.velma.waitForHP() != 0:
            raise Exception("Could not home head pan motor")

        self.velma.startHomingHT()
        if self.velma.waitForHT() != 0:
            raise Exception("Could not home head tilt motor")

        diag = self.velma.getCoreCsDiag()
        if not diag.motorsReady():
            raise Exception("Motors must be homed and ready to use for this test.")   

        self.trajectoryPlanner = Planner(self.velma.maxJointTrajLen())
        if not self.trajectoryPlanner.waitForInit():
            raise Exception("Could not initialize trajectory planner")
        self.octomapListener = OctomapListener("/octomap_binary")
        rospy.sleep(1.0)

        octomap = self.octomapListener.getOctomap(timeout_s=5.0)
        self.trajectoryPlanner.processWorld(octomap)

        self.catchingJointState = {
            'torso_0_joint':0,
            'right_arm_0_joint': 0.0,   
            'left_arm_0_joint':0.3,
            'right_arm_1_joint':-1.9,   
            'left_arm_1_joint':1.8,
            'right_arm_2_joint':1.5,   
            'left_arm_2_joint':-1.25,
            'right_arm_3_joint':1.5,   
            'left_arm_3_joint':-0.85,
            'right_arm_4_joint':0,      
            'left_arm_4_joint':0,
            'right_arm_5_joint':-1.6,   
            'left_arm_5_joint':0.5,
            'right_arm_6_joint':0.0,      
            'left_arm_6_joint': 0.0
        }
        self.catchingJointState2 = {
            'torso_0_joint':0,
            'left_arm_0_joint': -0.0,   
            'right_arm_0_joint':-0.3,
            'left_arm_1_joint':1.9,   
            'right_arm_1_joint':-1.8,
            'left_arm_2_joint':-1.5,   
            'right_arm_2_joint':1.25,
            'left_arm_3_joint':-1.5,   
            'right_arm_3_joint':0.85,
            'left_arm_4_joint':0,      
            'right_arm_4_joint':0,
            'left_arm_5_joint':1.6,   
            'right_arm_5_joint':-0.5,
            'left_arm_6_joint':0.0,      
            'right_arm_6_joint': 0.0
        }
        self.acquiringJointState = {
            'torso_0_joint': 0, 
            'right_arm_0_joint': 0.2176005457580103,   
            'left_arm_0_joint': 0.2345004080527655,
            'right_arm_1_joint': -1.9107791904878497,  
            'left_arm_1_joint': 1.8034769374904756,
            'right_arm_2_joint': 1.2409542924753767,   
            'left_arm_2_joint': -1.1982341925787994,
            'right_arm_3_joint': 1.4842204142092719,   
            'left_arm_3_joint': -0.8278483633253793, 
            'right_arm_4_joint': 0.2525831592128146,   
            'left_arm_4_joint': 0.07257063733648089,
            'right_arm_5_joint': -1.5390250000127208,  
            'left_arm_5_joint': 0.4699180006050142,
            'right_arm_6_joint': -0.21426825617036566, 
            'left_arm_6_joint': -0.0703725749418421,            
        }

        self.acquiringJointState2 = {
            'torso_0_joint': 0, 
            'left_arm_0_joint': -0.2176005457580103,   
            'right_arm_0_joint': -0.2345004080527655,
            'left_arm_1_joint': 1.9107791904878497,  
            'right_arm_1_joint': -1.8034769374904756,
            'left_arm_2_joint': -1.2409542924753767,   
            'right_arm_2_joint': 1.1982341925787994,
            'left_arm_3_joint': -1.4842204142092719,   
            'right_arm_3_joint': 0.8278483633253793, 
            'left_arm_4_joint': -0.2525831592128146,   
            'right_arm_4_joint': -0.07257063733648089,
            'left_arm_5_joint': 1.5390250000127208,  
            'right_arm_5_joint': -0.4699180006050142,
            'left_arm_6_joint': 0.31426825617036566, 
            'right_arm_6_joint': 0.0703725749418421,            
        }

        self.deliveringJointState =  {
            'torso_0_joint': 0.0,

            'left_arm_0_joint': 0.0,   
            'left_arm_1_joint':2.0,
            'left_arm_2_joint':-1.5, 
            'left_arm_3_joint':-1.65,   
            'left_arm_4_joint':0,
            'left_arm_5_joint':1.5,  
            'left_arm_6_joint': 0.0,

            'right_arm_0_joint':-0.3,   
            'right_arm_1_joint':-1.8,   
            'right_arm_2_joint':1.25,   
            'right_arm_3_joint':0.85,   
            'right_arm_4_joint':0,      
            'right_arm_5_joint':-0.5,   
            'right_arm_6_joint':0    
        }

        # self.grabbingJointState = {
        #   'left_arm_2_joint': -2.0355836822083657, 
        #   'right_arm_2_joint': 0.3709267067333251, 
        #   'right_arm_0_joint': 0.46569890384071055, 
        #   'right_arm_4_joint': 0.2241366773284909,  
        #   'right_arm_5_joint': -0.5316931184645763,
        #   'right_arm_3_joint': 0.38176870047166345, 
        #   'left_arm_0_joint': -0.15171555729988803, 
        #   'left_arm_6_joint': 0.4899765008800134,
        #   'right_arm_1_joint': -1.931444821966154, 
        #   'left_arm_1_joint': 1.9500022141952806, 
        #   'right_arm_6_joint': 0.188264943430854, 
        #   'torso_0_joint': -0.6293638853027678,
        #   'left_arm_3_joint': -1.0594496666438027, 
        #   'left_arm_5_joint': 1.7238910428595622, 
        #   'left_arm_4_joint': -0.06248503443106635
        # }

        # self.grabbingJointState = {
        #   'left_arm_2_joint': -1.3355836822083657, 
        #   'right_arm_2_joint': 0.3709267067333251, 
        #   'right_arm_0_joint': 0.46569890384071055, 
        #   'right_arm_4_joint': 0.2241366773284909,  
        #   'right_arm_5_joint': -0.5316931184645763,
        #   'right_arm_3_joint': 0.78176870047166345,
        #   'left_arm_0_joint': 0.20171555729988803, 
        #   'left_arm_6_joint': 0.5899765008800134,
        #   'right_arm_1_joint': -1.931444821966154, 
        #   'left_arm_1_joint': 1.9500022141952806, 
        #   'right_arm_6_joint': 0.188264943430854, 
        #   'torso_0_joint': -0.5293638853027678,
        #   'left_arm_3_joint': -1.6094496666438027, 
        #   'left_arm_5_joint': 1.7238910428595622, 
        #   'left_arm_4_joint': -0.06248503443106635
        # }

        self.grabbingJointState = {
            'torso_0_joint':-0.45,

            'left_arm_0_joint':-0.2,   
            'left_arm_1_joint':1.8,
            'left_arm_2_joint':-0.9, 
            'left_arm_3_joint':-1.65,   
            'left_arm_4_joint':0,
            'left_arm_5_joint':1.5,  
            'left_arm_6_joint': 0.8,

            'right_arm_0_joint':-0.3,   
            'right_arm_1_joint':-1.8,   
            'right_arm_2_joint':1.25,   
            'right_arm_3_joint':0.85,   
            'right_arm_4_joint':0,      
            'right_arm_5_joint':-0.5,   
            'right_arm_6_joint':0    
        }

        self.startingJointState = {
            'torso_0_joint':0,
            'right_arm_0_joint':-0.3,   
            'left_arm_0_joint':0.3,
            'right_arm_1_joint':-1.8,   
            'left_arm_1_joint':1.8,
            'right_arm_2_joint':1.25,   
            'left_arm_2_joint':-1.25,
            'right_arm_3_joint':0.85,   
            'left_arm_3_joint':-0.85,
            'right_arm_4_joint':0,      
            'left_arm_4_joint':0,
            'right_arm_5_joint':-0.5,   
            'left_arm_5_joint':0.5,
            'right_arm_6_joint':0,      
            'left_arm_6_joint': 0
        }
        self.startingLeftJointState = {
            'left_arm_0_joint':0.3,
            'left_arm_1_joint':1.8,
            'left_arm_2_joint':-1.25,
            'left_arm_3_joint':-0.85,
            'left_arm_4_joint':0,
            'left_arm_5_joint':0.5,
            'left_arm_6_joint': 0
        }

    def setJointImpedanceMode(self):
        self.velma.moveJointImpToCurrentPos(start_time=0.2)
        error = self.velma.waitForJoint()
        if error != 0:
            raise Exception("The action should have ended without error, but the error code is {}".format(error))

        rospy.sleep(0.5)
        diag = self.velma.getCoreCsDiag()
        if not diag.inStateJntImp():
            raise Exception("Core CS should be in joint impedance mode but it is not")

    def BaseLeftArm(self):
        self.setJointImpedanceMode()
        desiredJointState = copy.deepcopy(self.startingLeftJointState)
        self.velma.moveJoint(desiredJointState, 2.0, start_time=0.5, position_tol=10.0/180.0*math.pi)
        if self.velma.waitForJoint() != 0:
            raise Exception("Moving body failed")
    def setCartesianImpedanceMode(self):
        if not self.velma.moveCartImpRightCurrentPos(start_time=0.2):
            raise Exception("Could not set CartImp mode for right lwr")

        if self.velma.waitForEffectorRight() != 0:
            raise Exception("Right effector error")

        if not self.velma.moveCartImpLeftCurrentPos(start_time=0.2):
            raise Exception("Could not set CartImp mode for left lwr")

        if self.velma.waitForEffectorLeft() != 0:
            raise Exception("Left effector error")

        rospy.sleep(0.5)
        diag = self.velma.getCoreCsDiag()
        if not diag.inStateCartImp():
            raise Exception("Core CS should be in cartesian impedance mode but it is not")

    def moveBodyToStartingPosition(self):
        startingJointState = {
            'torso_0_joint':0,
            'right_arm_0_joint':-0.3,   
            'left_arm_0_joint':0.3,
            'right_arm_1_joint':-1.8,   
            'left_arm_1_joint':1.8,
            'right_arm_2_joint':1.25,   
            'left_arm_2_joint':-1.25,
            'right_arm_3_joint':0.85,   
            'left_arm_3_joint':-0.85,
            'right_arm_4_joint':0,      
            'left_arm_4_joint':0,
            'right_arm_5_joint':-0.5,   
            'left_arm_5_joint':0.5,
            'right_arm_6_joint':0,      
            'left_arm_6_joint': 0
        }

        self.velma.moveJoint(startingJointState, 3.0, start_time=0.5, position_tol=10.0/180.0*math.pi)
        if self.velma.waitForJoint() != 0:
            raise Exception("Moving body failed")

    def moveHeadToStartingPosition(self):
        startingHeadState = (0, 0)
        self.velma.moveHead(startingHeadState, 3.0, start_time=0.5)
        if self.velma.waitForHead() != 0:
            raise Exception("Moving head failed")
        rospy.sleep(0.5)

        if not isHeadConfigurationClose(self.velma.getHeadCurrentConfiguration(), startingHeadState, 0.1):
            raise Exception("Head too far from destination")

    def moveToStartingPosition(self):
        self.setJointImpedanceMode()
        self.moveBodyToStartingPosition()
        self.moveHeadToStartingPosition()

    def hideHands(self):
        handDesiredState = [0.5*math.pi, 0.5*math.pi, 0.5*math.pi, math.pi]
        self.velma.moveHandRight(handDesiredState, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
        self.velma.moveHandLeft(handDesiredState, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
        if self.velma.waitForHandRight() != 0 and self.velma.waitForHandLeft() != 0:
            raise Exception("Could not hide hands")
        self.setJointImpedanceMode()
    def hideHand(self, hand_name):
        handDesiredState = [0.5*math.pi, 0.5*math.pi, 0.5*math.pi, math.pi]
        if hand_name == 'right':
            self.velma.moveHandRight(handDesiredState, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
        if hand_name == 'left':
            self.velma.moveHandLeft(handDesiredState, [1, 1, 1, 1], [2000, 2000, 2000, 2000], 1000, hold=True)
        if self.velma.waitForHandRight() != 0 and self.velma.waitForHandLeft() != 0:
            raise Exception("Could not hide hands")
        self.setJointImpedanceMode()
    
    def prepareForGrip(self, destinationFrame):
        T_Wo_Cabinet = self.velma.getTf("Wo", destinationFrame)
        T_B_Cabinet = self.velma.getTf("B", destinationFrame)

        cabinetX = T_Wo_Cabinet.p[0]
        cabinetY = T_Wo_Cabinet.p[1]
        cabinetZ = T_Wo_Cabinet.p[2]

        torsoAngle = math.atan2(cabinetY, cabinetX)
        if torsoAngle > math.pi /2:
            torsoAngle = math.pi /2 - 0.1
        elif torsoAngle < -math.pi /2:
            torsoAngle = -math.pi /2 + 0.1
        else:
            pass

        desiredJointState = copy.deepcopy(self.acquiringJointState)
        desiredJointState["torso_0_joint"] = torsoAngle

        self.velma.moveJoint(desiredJointState, 2.0, start_time=0.5, position_tol=10.0/180.0*math.pi)

        if self.velma.waitForJoint() != 0:
            raise Exception("Could not move joints")

        rospy.sleep(0.5)
        lastJointState = self.velma.getLastJointState()
        if not isConfigurationClose(desiredJointState, lastJointState[1]):
            raise Exception("Could not acquire desired joint state")

        self.setCartesianImpedanceMode()
        return T_B_Cabinet

    def prepareForLeftOpenning(self, destinationFrame):
        T_Wo_Cabinet = self.velma.getTf("Wo", destinationFrame)
        T_B_Cabinet = self.velma.getTf("B", destinationFrame)

        cabinetX = T_Wo_Cabinet.p[0]
        cabinetY = T_Wo_Cabinet.p[1]
        cabinetZ = T_Wo_Cabinet.p[2]

        torsoAngle = math.atan2(cabinetY, cabinetX)
        if torsoAngle > math.pi /2:
            torsoAngle = math.pi /2 - 0.1
        elif torsoAngle < -math.pi /2:
            torsoAngle = -math.pi /2 + 0.1
        else:
            pass

        desiredJointState = copy.deepcopy(self.acquiringJointState2)
        desiredJointState["torso_0_joint"] = torsoAngle

        self.velma.moveJoint(desiredJointState, 2.0, start_time=0.5, position_tol=10.0/180.0*math.pi)

        if self.velma.waitForJoint() != 0:
            raise Exception("Could not move joints")

        rospy.sleep(0.5)
        lastJointState = self.velma.getLastJointState()
        if not isConfigurationClose(desiredJointState, lastJointState[1]):
            raise Exception("Could not acquire desired joint state")

        self.setCartesianImpedanceMode()
        return T_B_Cabinet


    def resolveRelativePosition(self, tf, deltaX, deltaY, deltaZ):
        (rotX, rotY, rotZ) = tf.M.GetRPY()
        posX = tf.p.x() + math.cos(rotZ)*deltaX - math.sin(rotZ)*deltaY
        posY = tf.p.y() + math.sin(rotZ)*deltaX + math.cos(rotZ)*deltaY
        posZ = tf.p.z() + deltaZ
        yaw = rotZ - math.pi
        if yaw < -math.pi:
            yaw += 2*math.pi
        return [posX, posY, posZ, yaw]

    def moveRelativeToInCartImpMode(self, tf, x, y, z, angle, tol=10):
        (setpointX, setpointY, setpointZ, setpointYaw) = self.resolveRelativePosition(tf, x, y, z)
        targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, setpointYaw+angle), PyKDL.Vector(setpointX, setpointY, setpointZ))
        result = self.moveInCartesianImpedanceMode(targetFrame, tol)
        return result

    def moveLeftRelativeToInCartImpMode(self, tf, x, y, z, angle, tol=10):
        (setpointX, setpointY, setpointZ, setpointYaw) = self.resolveRelativePosition(tf, x, y, z)
        targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, setpointYaw+angle), PyKDL.Vector(setpointX, setpointY, setpointZ))
        result = self.moveLeftInCartesianImpedanceMode(targetFrame, tol)
        return result

    def moveInCartesianImpedanceMode(self, T_B_dest, tol=10):
        if not self.velma.moveCartImpRight([T_B_dest], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, tol, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma.waitForEffectorRight() != 0:
            if not self.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
            return False
        else:
            return True

    def moveLeftInCartesianImpedanceMode(self, T_B_dest, tol=10):
        if not self.velma.moveCartImpLeft([T_B_dest], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, tol, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma.waitForEffectorLeft() != 0:
            if not self.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
            return False
        else:
            return True

    def setRightLWRImpedance(self, imp_p_x, imp_p_y, imp_p_z, imp_r_x, imp_r_y, imp_r_z):
        if not self.velma.moveCartImpRight(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(imp_p_x, imp_p_y, imp_p_z), PyKDL.Vector(imp_r_x, imp_r_y, imp_r_z))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            raise Exception("setRightLWRImpedance -> Could not change impedance of right lwr")
        # if self.velma.waitForEffectorRight() == 0:
        #     raise Exception("setRightLWRImpedance -> Waiting for right lwr failed")
        rospy.sleep(1)
    def setLeftLWRImpedance(self, imp_p_x, imp_p_y, imp_p_z, imp_r_x, imp_r_y, imp_r_z):
        if not self.velma.moveCartImpLeft(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(imp_p_x, imp_p_y, imp_p_z), PyKDL.Vector(imp_r_x, imp_r_y, imp_r_z))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            raise Exception("Could not change impedance of left lwr")
        if self.velma.waitForEffectorLeft() != 0:
            raise Exception("Could not change impedance of left lwr")
        rospy.sleep(1)

    def moveHeadTo(self, headPan, headTilt):
        desiredHeadState = (headPan, headTilt)
        self.velma.moveHead(desiredHeadState, 3.0, start_time=0.5)
        if self.velma.waitForHead() != 0:
            raise Exception("Moving head failed")
        rospy.sleep(0.5)

        if not isHeadConfigurationClose(self.velma.getHeadCurrentConfiguration(), desiredHeadState, 0.1):
            raise Exception("Head too far from destination")

    def moveTorso(self, angle):
        q = self.velma.getLastJointState()
        q[1]['torso_0_joint'] = angle
        self.velma.moveJoint(q[1], 15.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
        error = self.velma.waitForJoint()
        if error != 0:
            print "The action should have ended without error, but the error code is", error
            exitError(6)


    def lookForObject(self):
        self.moveHeadTo(0.0, 0.0)
        self.moveHeadTo(0.0, 0.9)
        self.moveHeadTo(0.0, 0.0)
        # self.moveTorso(0.7)
        # self.moveHeadTo(1.0, 0.7)
        # self.moveTorso(-0.7)
        # self.moveHeadTo(0.0, 0.7)
        # self.moveTorso(0.0)
        # self.moveHeadTo(0.0, 0.0)

        octomap = self.octomapListener.getOctomap(timeout_s=5.0)
        self.trajectoryPlanner.processWorld(octomap)

    def moveInJointImpedanceModeWithPlan(self, desiredJointState):
        goalConstraint = qMapToConstraints(desiredJointState, 0.01, group=self.velma.getJointGroup("impedance_joints"))
        for i in range(5):
            rospy.sleep(0.5)
            lastJointState = self.velma.getLastJointState()
            print("Planning try no. {}".format(i))
            trajectory = self.trajectoryPlanner.plan(lastJointState[1], [goalConstraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if trajectory is None:
                continue

            print("Found trajectory, executing...")
            if not self.velma.moveJointTraj(trajectory, start_time=0.5):
                raise Exception("Could not move velma")
            if self.velma.waitForJoint() == 0:
                break
            else:
                print("Trajectory could not be completed")
                continue

        print("Trajectory executed")
        rospy.sleep(0.5)
        lastJointState = self.velma.getLastJointState()
        print("Trajectory executed dsds ")
        if not isConfigurationClose(desiredJointState, lastJointState[1]):
            raise Exception("Could not move joints to desired state")
        print("Trajectory executed xxx")

    def moveToInteractiveCursor(self):
        T_Wo_test = self.velma.getTf("Wo", "example_frame")
        if not self.velma.moveCartImpLeftCurrentPos(start_time=0.2):
            exitError(8)
        if self.velma.waitForEffectorLeft() != 0:
            exitError(9)
        if not self.velma.moveCartImpLeft([T_Wo_test], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
        if self.velma.waitForEffectorLeft() != 0:
            exitError(9)
        rospy.sleep(0.5)

    def openLeftHand(self):
        dest_q = [0.80, 0.80, 0.7, 0]
        self.velma.moveHandLeft(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if self.velma.waitForHandLeft() != 0:
            raise Exception("Hand error")
        rospy.sleep(0.5)
        if not isHandConfigurationClose(self.velma.getHandLeftCurrentConfiguration(), dest_q):
            raise Exception("Hand configuration")

    def openRightHand(self):
        dest_q = [0.80, 0.80, 0.7, 0]
        self.velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if self.velma.waitForHandRight() != 0:
            raise Exception("Right hand error")
        rospy.sleep(0.5)
        if not isHandConfigurationClose(self.velma.getHandRightCurrentConfiguration(), dest_q):
            raise Exception("Right hand configuration")

    def prepareForCatch(self):
        T_B_Wr = self.velma.getTf("B", "Wl")
        T_Wr_Gr = self.velma.getTf("Wl", "Gl")
        if not self.velma.moveCartImpLeft([T_B_Wr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        if self.velma.waitForEffectorLeft() != 0:
            exitError(19)
        rospy.sleep(0.5)

    def moveLeftRelativeToInCartImpMode(self, tf, x, y, z, angle, tol=10):
        (setpointX, setpointY, setpointZ, setpointYaw) = self.resolveRelativePosition(tf, x, y, z)
        targetFrame = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, setpointYaw+angle), PyKDL.Vector(setpointX, setpointY, setpointZ))
        result = self.moveLeftInCartesianImpedanceMode(targetFrame, tol)
        return result

    def moveLeftInCartesianImpedanceMode(self, T_B_dest, tol=10):    
        if not self.velma.moveCartImpLeft([T_B_dest], [10.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, tol, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma.waitForEffectorLeft() != 0:
            if not self.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
            return False
        else:
            return True

    def moveMobileBase(self, x_vel, y_vel, theta_vel, time):
        publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        msg = Twist()
        msg.linear.x = x_vel
        msg.linear.y = y_vel
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = theta_vel
        startTime = rospy.get_rostime()
        while rospy.get_rostime() - startTime < rospy.Duration.from_sec(time):
            publisher.publish(msg)

        stopMsg = Twist()
        publisher.publish(stopMsg)


class PrepareToMoveBase(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Przygotowuję się do ruchu bazy'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Przygotowuję się do ruchu bazy' )
        print("Reset robot position")
        self.velma_task_executor.moveToStartingPosition()

        print("Hiding both hands")
        self.velma_task_executor.hideHands()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class PrepareToMoveWithObject(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Przygotowuję się do ruchu bazy'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Przygotowuję się do ruchu bazy' )
        print("Reset robot position")
        self.velma_task_executor.moveToStartingPosition()

        # print("Hiding both hands")
        # self.velma_task_executor.hideHands()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class HideHands(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Chowam dlonie'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Chowam dlonie' )

        print("Hiding both hands")
        self.velma_task_executor.hideHands()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class PrepareToGrip(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=['object_container'], output_keys=['object_container_grab_pose'],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Przygotowuję się do chwytu'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Przygotowuję się do chwytu' )
        print("Preparing for grip")
        userdata.object_container_grab_pose = self.velma_task_executor.prepareForGrip(userdata.object_container)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'


class OpenDoor(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=['object_container_grab_pose','object_container'], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Otwieram drzwi'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Otwieram drzwi' )
        
        print("Moving closer to cabinet")
        self.velma_task_executor.moveRelativeToInCartImpMode(userdata.object_container_grab_pose, 0.40, -0.2, 0.09, 0.0)

        print("Finding the door")
        self.velma_task_executor.setRightLWRImpedance(500, 500, 900, 1000, 1000, 200)
        self.velma_task_executor.moveRelativeToInCartImpMode(
            userdata.object_container_grab_pose, 0.25, -0.2, 0.09, 0.0)

        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'

        print("Finding the door handle")
        self.velma_task_executor.moveRelativeToInCartImpMode(
                userdata.object_container_grab_pose, 0.27, -0.38, 0.09, 0.0)
        # self.velma_task_executor.moveRelativeToInCartImpMode(
        #         userdata.object_container_grab_pose, 0.37, -0.33, 0.09, 0.0)

        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        print("Open a lil bit")
        self.velma_task_executor.setRightLWRImpedance(200, 200, 900, 1000, 1000, 200)
        self.velma_task_executor.moveRelativeToInCartImpMode(
                userdata.object_container_grab_pose, 0.50, -0.35, 0.09, 0.0)
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        print("Open a lil bit more")
        self.velma_task_executor.setRightLWRImpedance(300, 300, 900, 200, 200, 200)
        self.velma_task_executor.moveRelativeToInCartImpMode(
                userdata.object_container_grab_pose, 0.65, -0.1, 0.09, 0.0)
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        print("Open a lil bit more")
        self.velma_task_executor.setRightLWRImpedance(300, 300, 900, 200, 200, 200)
        self.velma_task_executor.moveRelativeToInCartImpMode(
                userdata.object_container_grab_pose, 0.65, 0, 0.09, 0.0)
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        # self.velma_task_executor.moveMobileBase(0.0, -0.1, 0.0, 1.5)

        # print("Release handle")
        # self.velma_task_executor.setRightLWRImpedance(300, 300, 900, 200, 200, 200)
        # self.velma_task_executor.moveRelativeToInCartImpMode(
        #         userdata.object_container_grab_pose, 0.4, 0.1, 0.09, 0.0)
        # self.velma_task_executor.moveToStartingPosition()

        print("Prepare Left")
        object_pose = self.velma_task_executor.prepareForLeftOpenning('object_corrected')
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        print("Move Left")
        if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.2):
          raise Exception("Could not set CartImp mode for left lwr")

        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
          raise Exception("Left effector error")

        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)

        objectTF = self.velma_task_executor.velma.getTf("B", "object_corrected")
        self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.4, 0.15, 0.08, 0)
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, -0.1, 0.08, 0)
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        # self.velma_task_executor.moveToStartingPosition()
        # self.velma_task_executor.setLeftLWRImpedance(300, 300, 900, 200, 200, 200)
        # self.velma_task_executor.moveLeftRelativeToInCartImpMode(
        #          userdata.object_container_grab_pose, 0.1, -0.05, 0.09, 0.0)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class CorrectBasePose(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Poprawiam pozycję bazy'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Poprawiam pozycję bazy' )
        
        self.velma_task_executor.moveToStartingPosition()
        self.velma_task_executor.moveMobileBase(-0.2, 0.0, 0.0, 2.0)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class OpenMore(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=['object_container_grab_pose'], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Otwieram drzwi'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Otwieram drzwi' )
        
        print("Moving closer to cabinet")
        self.velma_task_executor.setRightLWRImpedance(300, 300, 900, 200, 200, 200)
        self.velma_task_executor.moveRelativeToInCartImpMode(
                userdata.object_container_grab_pose, -0.3, -0.1, 0.09, 0.0)
        self.velma_task_executor.moveRelativeToInCartImpMode(
                userdata.object_container_grab_pose, 0.1, -0.1, 0.09, 0.0)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class LookForObject(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Szukam obiektu'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Szukam obiektu' )
        
        
        print("Looking for object")
        self.velma_task_executor.lookForObject()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class ApproachObject(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Podchodzę ramieniem do obiektu'
        self.velma_task_executor = velma_task_executor
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Podchodzę ramieniem do obiektu' )
        
        
        self.velma_task_executor.hideHand('left')
        print("Moving to grabbing position with planner")
        self.velma_task_executor.setJointImpedanceMode()
        desiredJointState = copy.deepcopy(self.velma_task_executor.catchingJointState)
        self.velma_task_executor.moveInJointImpedanceModeWithPlan(desiredJointState)

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class TakeOutObject(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor, marker_publisher):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Wyjmuję obiekt'
        self.velma_task_executor = velma_task_executor
        self.marker_publisher = marker_publisher
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Wyjmuję obiekt' )
        
        print("Grabbing")
        print("Set cimp mode for left lwr")
        # self.velma_task_executor.BaseLeftArm()
        # print("Prepare Left")
        # object_pose = self.velma_task_executor.prepareForLeftOpenning('object_corrected')
        # if self.is_suspension_flag() is not None:
        #     print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
        #     print "Moving to starting position and getting suspending plan"
        #     self.velma_task_executor.moveToStartingPosition()
        #     return 'preemption'
        print("Prepare Right")
        userdata.object_container_grab_pose = self.velma_task_executor.prepareForGrip('object_corrected')
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.2):
          raise Exception("Could not set CartImp mode for left lwr")

        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
          raise Exception("Right effector error")

        self.velma_task_executor.setRightLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)
        print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
        T_B_Wl = self.velma_task_executor.velma.getTf("B", "Wr")
        T_Wl_Gl = self.velma_task_executor.velma.getTf("Wr", "Gr")
        if not self.velma_task_executor.velma.moveCartImpRight([T_B_Wl*T_Wl_Gl], [0.1], [T_Wl_Gl], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        # if self.velma_task_executor.velma.waitForEffectorRight() != 0:
        #     exitError(19)
        print "The right tool is now in 'grip' pose"
        rospy.sleep(0.5)


        tableTF = self.velma_task_executor.velma.getTf("B", "object_corrected")
        x = tableTF.p[0]
        y = tableTF.p[1]
        z = tableTF.p[2]
        
        # rot = PyKDL.Rotation.RPY( 1.540, -1.540, 0)
        rot = PyKDL.Rotation.RPY(0,1.54,0)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.5, y+0.03, z))

        
        print("Move infront object")
        tol = 0.1
        # self.velma_task_executor.setRightLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")      
        print("Open hand") # CZEKAJ 
        self.velma_task_executor.openRightHand()
        rospy.sleep(1.0)  
        print("Approach object")
        rot = PyKDL.Rotation.RPY(0,1.54,0)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x+0.01, y, z-0.0))
        t_broadcaster = tf2_ros.TransformBroadcaster()
        object_grab_tf = TransformStamped()
        object_pose = pm.toMsg(BT)
        object_grab_tf.header.stamp = rospy.Time.now()
        object_grab_tf.header.frame_id = 'torso_base'
        object_grab_tf.child_frame_id = 'object_grab'
        object_grab_tf.transform.translation = object_pose.position
        object_grab_tf.transform.rotation = object_pose.orientation
        i=0
        while i<10:
            t_broadcaster.sendTransform(object_grab_tf)
            rospy.sleep(0.2)
            i = i+1
        tol = 15
        self.velma_task_executor.setRightLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1,path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, tol, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
        # rospy.sleep(5)
        print("Close hand on object")
        handDesiredState = dest_q = [80.0/180.0*math.pi, 80.0/180.0*math.pi, 80.0/180.0*math.pi, 0]
        self.velma_task_executor.velma.moveHandRight(handDesiredState, [1,1,1,1], [1000, 1000, 1000, 1000], 1000, hold=True)
        if self.velma_task_executor.velma.waitForHandRight() != 0:
          raise Exception("Could not catch with left hand")
        i=0
        while i< 5:
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            i= i +1
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.openRightHand()
            rospy.sleep(1.0)
            self.velma_task_executor.moveRelativeToInCartImpMode(objectTF, -0.5, 0, 0.09, 0)
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        print("Lift object")
        rot = PyKDL.Rotation.RPY(0,1.54,0)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x+0.08, y, z+0.1))
        tol = 10.0
        self.velma_task_executor.setRightLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
        print("get out object")
        rot = PyKDL.Rotation.RPY(0,1.54,0)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.65, y, z+0.1))
        tol = 10.0
        self.velma_task_executor.setRightLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
        

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

# class TakeOutObjectLeft(TaskER.BlockingState):
#     def __init__(self, sim_mode, conversation_interface, velma_task_executor, marker_publisher):
#         TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
#                              outcomes=['ok', 'preemption', 'error', 'shutdown'])

#         self.conversation_interface = conversation_interface

#         self.description = u'Wyjmuję obiekt'
#         self.velma_task_executor = velma_task_executor
#         self.marker_publisher = marker_publisher
#     def transition_function(self, userdata):
#         rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
#         #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
#         self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Wyjmuję obiekt' )
        
#         print("Grabbing")
#         print("Set cimp mode for left lwr")
#         # self.velma_task_executor.BaseLeftArm()

#         if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.2):
#           raise Exception("Could not set CartImp mode for left lwr")

#         if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
#           raise Exception("Left effector error")

#         self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)

#         objectTF = self.velma_task_executor.velma.getTf("B", "object_corrected")
#         self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, 0, 0.08, 0)

#         if self.is_suspension_flag() is not None:
#             print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
#             print "Moving to starting position and getting suspending plan"
#             self.velma_task_executor.moveToStartingPosition()
#             return 'preemption'
#         # self.velma_task_executor.hideHands()
#         print("Open hand") # CZEKAJ 
#         self.velma_task_executor.openLeftHand()
#         rospy.sleep(1.0)
#         self.velma_task_executor.setCartesianImpedanceMode()
#         # self.velma_task_executor.setCartesianImpedanceMode()
#         self.velma_task_executor.setLeftLWRImpedance(300, 1000, 1000, 2000, 1000, 1000)
#         self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.08, 0, 0.05, 0)

#         if self.is_suspension_flag() is not None:
#             print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
#             print "Moving to starting position and getting suspending plan"
#             self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, 0, 0.09, 0)
#             self.velma_task_executor.moveToStartingPosition()
#             return 'preemption'

#         print("Add collision")
#         self.marker_publisher.start()

#         print("Close hand on object")
#         handDesiredState = dest_q = [74.0/180.0*math.pi, 74.0/180.0*math.pi, 74.0/180.0*math.pi, 0]
#         self.velma_task_executor.velma.moveHandLeft(handDesiredState, [1,1,1,1], [1000, 1000, 1000, 1000], 1000, hold=True)
#         if self.velma_task_executor.velma.waitForHandLeft() != 0:
#           raise Exception("Could not catch with left hand")
#         i=0
#         while i< 5:
#             print "HAND CLOSED"
#             print "HAND CLOSED"
#             print "HAND CLOSED"
#             print "HAND CLOSED"
#             print "HAND CLOSED"
#             print "HAND CLOSED"
#             print "HAND CLOSED"
#             i= i +1
#         if self.is_suspension_flag() is not None:
#             print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
#             print "Moving to starting position and getting suspending plan"
#             self.velma_task_executor.openLeftHand()
#             rospy.sleep(1.0)
#             self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, 0, 0.09, 0)
#             self.velma_task_executor.moveToStartingPosition()
#             return 'preemption'

#         print("lift")
#         self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.1, 0, 0.1, 0)
#         if self.is_suspension_flag() is not None:
#             print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
#             print "Moving to starting position and getting suspending plan"
#             self.velma_task_executor.openLeftHand()
#             rospy.sleep(1.0)
#             self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, 0, 0.09, 0)
#             self.velma_task_executor.moveToStartingPosition()
#             return 'preemption'
           
#         print("pull hand back")
#         self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.75, 0, 0.1, 0)
        

#         if self.is_suspension_flag() is not None:
#             print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
#             print "Moving to starting position and getting suspending plan"
#             self.velma_task_executor.moveToStartingPosition()
#             return 'preemption'

#         if self.__shutdown__:
#             return 'shutdown'
#         return 'ok'

class PrepareTakeOutObjectLeft(TaskER.SuspendableState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor, marker_publisher):
        TaskER.SuspendableState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Przygotowuję się do wyjęcia obiektu'
        self.velma_task_executor = velma_task_executor
        self.marker_publisher = marker_publisher
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Przygotowuję się do wyjęcia obiektu' )
        
        print("Grabbing")
        print("Set cimp mode for left lwr")
        # self.velma_task_executor.BaseLeftArm()
        # print("Prepare Left")
        # object_pose = self.velma_task_executor.prepareForLeftOpenning('object_corrected')
        # if self.is_suspension_flag() is not None:
        #     print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
        #     print "Moving to starting position and getting suspending plan"
        #     self.velma_task_executor.moveToStartingPosition()
        #     return 'preemption'
        print("Prepare Left")
        object_pose = self.velma_task_executor.prepareForLeftOpenning('object_corrected')
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.2):
          raise Exception("Could not set CartImp mode for left lwr")

        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
          raise Exception("Left effector error")

        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            return 'preemption'

        self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)
        print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
        T_B_Wl = self.velma_task_executor.velma.getTf("B", "Wl")
        T_Wl_Gl = self.velma_task_executor.velma.getTf("Wl", "Gl")
        if not self.velma_task_executor.velma.moveCartImpLeft([T_B_Wl*T_Wl_Gl], [0.1], [T_Wl_Gl], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        # if self.velma_task_executor.velma.waitForEffectorRight() != 0:
        #     exitError(19)
        print "The right tool is now in 'grip' pose"
        rospy.sleep(0.5)

        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            return 'preemption'


        tableTF = self.velma_task_executor.velma.getTf("B", "object_corrected")
        x = tableTF.p[0]
        y = tableTF.p[1]
        z = tableTF.p[2]
        
        # rot = PyKDL.Rotation.RPY( 1.540, -1.540, 0)
        rot = PyKDL.Rotation.RPY( -1.540, -1.540, -1.540)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.5, y+0.03, z))

        
        print("Move infront object")
        tol = 5
        self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 200, 2000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")   

        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            return 'preemption'

        print("Open hand") # CZEKAJ 
        self.velma_task_executor.openLeftHand()
        rospy.sleep(1.0)  

        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            return 'preemption'

        return 'ok'

class TakeOutObjectLeft(TaskER.BlockingState):

    def __init__(self, sim_mode, conversation_interface, velma_task_executor, marker_publisher, pulled_out_flag):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Wyjmuję obiekt'
        self.velma_task_executor = velma_task_executor
        self.marker_publisher = marker_publisher
        self.pulled_out_flag = pulled_out_flag
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Wyjmuję obiekt' )

        print("Approach object")        
        tableTF = self.velma_task_executor.velma.getTf("B", "object_corrected")
        x = tableTF.p[0]
        y = tableTF.p[1]
        z = tableTF.p[2]

        rot = PyKDL.Rotation.RPY( -1.540, -1.540, -1.540)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x+0.0, y, z-0.0))
        t_broadcaster = tf2_ros.TransformBroadcaster()
        object_grab_tf = TransformStamped()
        object_pose = pm.toMsg(BT)
        object_grab_tf.header.stamp = rospy.Time.now()
        object_grab_tf.header.frame_id = 'torso_base'
        object_grab_tf.child_frame_id = 'object_grab'
        object_grab_tf.transform.translation = object_pose.position
        object_grab_tf.transform.rotation = object_pose.orientation
        i=0
        while i<10:
            t_broadcaster.sendTransform(object_grab_tf)
            rospy.sleep(0.2)
            i = i+1
        tol = 5
        self.velma_task_executor.setLeftLWRImpedance(2000, 2000, 2000, 2000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1,path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, tol, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
        # rospy.sleep(5)

        print("Close hand on object")
        handDesiredState = dest_q = [70.0/180.0*math.pi, 70.0/180.0*math.pi, 70.0/180.0*math.pi, 0]
        self.velma_task_executor.velma.moveHandLeft(handDesiredState, [1,1,1,1], [1000, 1000, 1000, 1000], 1000, hold=True)
        if self.velma_task_executor.velma.waitForHandLeft() != 0:
          raise Exception("Could not catch with left hand")
        i=0
        while i< 5:
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            print "HAND CLOSED"
            i= i +1
        if self.is_suspension_flag() is not None:
            print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
            print "Moving to starting position and getting suspending plan"
            self.velma_task_executor.openLeftHand()
            rospy.sleep(1.0)
            self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, 0, 0.09, 0)
            self.velma_task_executor.moveToStartingPosition()
            return 'preemption'
        print("Lift object")
        rot = PyKDL.Rotation.RPY( -1.540, -1.540, -1.540)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x+0.0, y, z+0.04))
        tol = 10.0
        self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 1000, 2000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
        print("get out object")
        rot = PyKDL.Rotation.RPY( -1.540, -1.540, -1.540)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.65, y, z+0.06))
        tol = 10.0
        self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 200, 2000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")

        self.pulled_out_flag.set(True)

        # objectTF = self.velma_task_executor.velma.getTf("B", "object_corrected")
        # self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, 0, 0.03, 0)

        # if self.is_suspension_flag() is not None:
        #     print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
        #     print "Moving to starting position and getting suspending plan"
        #     self.velma_task_executor.moveToStartingPosition()
        #     return 'preemption'
        # # self.velma_task_executor.hideHands()
        # print("Open hand") # CZEKAJ 
        # self.velma_task_executor.openLeftHand()
        # rospy.sleep(1.0)
        # self.velma_task_executor.setCartesianImpedanceMode()
        # # self.velma_task_executor.setCartesianImpedanceMode()
        # self.velma_task_executor.setLeftLWRImpedance(300, 1000, 1000, 2000, 1000, 1000)
        # self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.0, 0, 0.0, 0)

        # if self.is_suspension_flag() is not None:
        #     print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
        #     print "Moving to starting position and getting suspending plan"
        #     self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, 0, 0.09, 0)
        #     self.velma_task_executor.moveToStartingPosition()
        #     return 'preemption'

        # print("Add collision")
        # self.marker_publisher.start()

        # print("Close hand on object")
        # handDesiredState = dest_q = [74.0/180.0*math.pi, 74.0/180.0*math.pi, 74.0/180.0*math.pi, 0]
        # self.velma_task_executor.velma.moveHandLeft(handDesiredState, [1,1,1,1], [1000, 1000, 1000, 1000], 1000, hold=True)
        # if self.velma_task_executor.velma.waitForHandLeft() != 0:
        #   raise Exception("Could not catch with left hand")
        # i=0
        # while i< 5:
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     i= i +1
        # if self.is_suspension_flag() is not None:
        #     print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
        #     print "Moving to starting position and getting suspending plan"
        #     self.velma_task_executor.openLeftHand()
        #     rospy.sleep(1.0)
        #     self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, 0, 0.09, 0)
        #     self.velma_task_executor.moveToStartingPosition()
        #     return 'preemption'

        # print("lift")
        # self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0, 0, 0.03, 0)
        # if self.is_suspension_flag() is not None:
        #     print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
        #     print "Moving to starting position and getting suspending plan"
        #     self.velma_task_executor.openLeftHand()
        #     rospy.sleep(1.0)
        #     self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.5, 0, 0.09, 0)
        #     self.velma_task_executor.moveToStartingPosition()
        #     return 'preemption'
           
        # print("pull hand back")
        # self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.65, 0, 0.1, 0)
        

        # if self.is_suspension_flag() is not None:
        #     print "TF GOT SUSPENSION FLAG: ", self.is_suspension_flag()
        #     print "Moving to starting position and getting suspending plan"
        #     self.velma_task_executor.moveToStartingPosition()
        #     return 'preemption'

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'
        


class PutDownObject(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor, marker_publisher):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Odstawiam obiekt'
        self.velma_task_executor = velma_task_executor
        self.marker_publisher = marker_publisher
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Odstawiam obiekt' )
        object_pose = self.velma_task_executor.prepareForLeftOpenning('stolik')

        print("Set cimp mode for right lwr")
        if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.2):
            raise Exception("Could not set CartImp mode for right lwr")

        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            raise Exception("Right effector error")

        print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
        T_B_Wl = self.velma_task_executor.velma.getTf("B", "Wr")
        T_Wl_Gl = self.velma_task_executor.velma.getTf("Wr", "Gr")
        if not self.velma_task_executor.velma.moveCartImpRight([T_B_Wl*T_Wl_Gl], [0.1], [T_Wl_Gl], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        # if self.velma_task_executor.velma.waitForEffectorRight() != 0:
        #     exitError(19)
        print "The right tool is now in 'grip' pose"
        rospy.sleep(0.5)

        tableTF = self.velma_task_executor.velma.getTf("B", "stolik_corrected")
        x = tableTF.p[0]
        y = tableTF.p[1]
        z = tableTF.p[2]
        
        rot = PyKDL.Rotation.RPY(0.0, 1.54, 0.0)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.20, y, z+1.2))

        
        print("Move hand over the table")
        tol = 10.0
        self.velma_task_executor.setRightLWRImpedance(1000, 1000, 200, 2000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")



        print("Put can on the table")
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.20, y, z+0.5))
        tol = 10.0
        self.velma_task_executor.setRightLWRImpedance(200, 200, 500, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")

        

        print("open hand")
        dest_q = [0.0, 0.0, 0.0, 0]
        self.velma_task_executor.velma.moveHandRight(dest_q, [10,10,10,10], [2000, 2000, 2000, 2000], 1000, hold=True)
        if self.velma_task_executor.velma.waitForHandRight() != 0:
            raise Exception("Hand error")
            rospy.sleep(0.5)

        print("pull back")
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.30, y, z+0.9))
        tol = 10.0
        self.velma_task_executor.setRightLWRImpedance(200, 200, 200, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpRight([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorRight() != 0:
            if not self.velma_task_executor.velma.moveCartImpRightCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
        
        self.velma_task_executor.moveMobileBase(-0.2, 0.0, 0.0, 1.0)
        self.velma_task_executor.hideHands()
        self.velma_task_executor.moveToStartingPosition()
        self.marker_publisher.stop()

        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class PutDownObjectLeft(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor, marker_publisher):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Odstawiam obiekt'
        self.velma_task_executor = velma_task_executor
        self.marker_publisher = marker_publisher
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Odstawiam obiekt' )

        object_pose = self.velma_task_executor.prepareForLeftOpenning('stolik')

        print("Set cimp mode for left lwr")
        self.velma_task_executor.setJointImpedanceMode()
        if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.2):
            raise Exception("Could not set CartImp mode for Left lwr")

        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            raise Exception("Left effector error")


        print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
        T_B_Wl = self.velma_task_executor.velma.getTf("B", "Wl")
        T_Wl_Gl = self.velma_task_executor.velma.getTf("Wl", "Gl")
        if not self.velma_task_executor.velma.moveCartImpLeft([T_B_Wl*T_Wl_Gl], [0.1], [T_Wl_Gl], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        # if self.velma_task_executor.velma.waitForEffectorRight() != 0:
        #     exitError(19)
        print "The right tool is now in 'grip' pose"
        rospy.sleep(0.5)

        # if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.2):
        #   raise Exception("Could not set CartImp mode for left lwr")

        # if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
        #   raise Exception("Left effector error")

        # self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 1000, 1000, 1000, 1000)

        # print "Move above stolik"
        # objectTF = self.velma_task_executor.velma.getTf("B", "stolik_corrected")
        # self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, 0.1, 0, 1.2, 0)

        # # self.velma_task_executor.hideHands()
        # print "Set mode"
        # self.velma_task_executor.setCartesianImpedanceMode()
        # # self.velma_task_executor.setCartesianImpedanceMode()
        # self.velma_task_executor.setLeftLWRImpedance(300, 1000, 1000, 2000, 1000, 1000)
        # print "Putting down"
        # self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, 0.1, 0, 0.6, 0)

        # print("Close hand on object")
        # handDesiredState = dest_q = [74.0/180.0*math.pi, 74.0/180.0*math.pi, 74.0/180.0*math.pi, 0]
        # self.velma_task_executor.velma.moveHandLeft(handDesiredState, [1,1,1,1], [1000, 1000, 1000, 1000], 1000, hold=True)
        # if self.velma_task_executor.velma.waitForHandLeft() != 0:
        #   raise Exception("Could not catch with left hand")

        # print("Open hand on object")
        # # print("Open hand") # CZEKAJ 
        # # self.velma_task_executor.openLeftHand()
        # # rospy.sleep(3.0)
        # handDesiredState = dest_q = [10.0/180.0*math.pi, 10.0/180.0*math.pi, 10.0/180.0*math.pi, 0]
        # self.velma_task_executor.velma.moveHandLeft(handDesiredState, [1,1,1,1], [1000, 1000, 1000, 1000], 1000, hold=True)
        # if self.velma_task_executor.velma.waitForHandLeft() != 0:
        #   raise Exception("Could not catch with left hand")
        # i=0
        # while i< 5:
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     print "HAND CLOSED"
        #     i= i +1
        # self.velma_task_executor.hideHands()
        # print("lift")
        # self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.18, 0, 0.15, 0)
           
        # print("pull hand back")
        # self.velma_task_executor.moveLeftRelativeToInCartImpMode(objectTF, -0.95, 0, 0.15, 0)

        # self.velma_task_executor.moveMobileBase(-0.0, -0.2, 0.0, 1.0)
        # self.velma_task_executor.moveToStartingPosition()


        tableTF = self.velma_task_executor.velma.getTf("B", "stolik_corrected")
        x = tableTF.p[0]
        y = tableTF.p[1]
        z = tableTF.p[2]
        
        # rot = PyKDL.Rotation.RPY( 1.540, -1.540, 0)
        rot = PyKDL.Rotation.RPY( -1.540, -1.540, -1.540)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.4, y-0.1, z+1))

        
        print("Move hand over the table")
        tol = 10.0
        self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 200, 2000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")



        print("Put can on the table")
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.4, y-0.1, z+0.8))
        tol = 10.0
        self.velma_task_executor.setLeftLWRImpedance(200, 200, 500, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")

        


        print("open hand")
        dest_q = [0.0, 0.0, 0.0, 0]
        self.velma_task_executor.velma.moveHandLeft(dest_q, [1,1,1,1], [500, 500, 500, 500], 500, hold=True)
        if self.velma_task_executor.velma.waitForHandLeft() != 0:
            raise Exception("Hand error")
            rospy.sleep(0.5)
        print("release object")
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.5, y-0.1, z+0.9))
        tol = 10.0
        self.velma_task_executor.setLeftLWRImpedance(200, 200, 500, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [2.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")
        print("Move base")

        print("pull back")
        # BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.4, y-0.1, z+1.2))
        # tol = 10.0
        # if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
        #     raise Exception("Could not move in cartesian impedance mode")
        # if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
        #     if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
        #         raise Exception("Could not make it to given position")
        print("Move base")
        self.velma_task_executor.setJointImpedanceMode()
        self.velma_task_executor.moveMobileBase(-0.1, 0.0, 0.0, 5.0)
        print("Hide hands")
        self.velma_task_executor.hideHands()
        print("Set move pose")
        self.velma_task_executor.moveToStartingPosition()


        if self.__shutdown__:
            return 'shutdown'
        return 'ok'

class EmergencyPutDownObject(TaskER.BlockingState):
    def __init__(self, sim_mode, conversation_interface, velma_task_executor, marker_publisher):
        TaskER.BlockingState.__init__(self,tf_freq=10, input_keys=[], output_keys=[],
                             outcomes=['ok', 'preemption', 'error', 'shutdown'])

        self.conversation_interface = conversation_interface

        self.description = u'Odstawiam obiekt'
        self.velma_task_executor = velma_task_executor
        self.marker_publisher = marker_publisher
    def transition_function(self, userdata):
        rospy.loginfo('{}: Executing state: {}'.format(rospy.get_name(), self.__class__.__name__))
        #self.conversation_interface.addSpeakSentence( u'Zakończyłem zadanie' )
        self.conversation_interface.speakNowBlocking( u'niekorzystne warunki pogodowe Odstawiam obiekt' )

        object_pose = self.velma_task_executor.prepareForLeftOpenning('szafka_above')

        print("Set cimp mode for left lwr")
        self.velma_task_executor.setJointImpedanceMode()
        if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.2):
            raise Exception("Could not set CartImp mode for Left lwr")

        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            raise Exception("Left effector error")

        print "Moving the right tool and equilibrium pose from 'wrist' to 'grip' frame..."
        T_B_Wl = self.velma_task_executor.velma.getTf("B", "Wl")
        T_Wl_Gl = self.velma_task_executor.velma.getTf("Wl", "Gl")
        if not self.velma_task_executor.velma.moveCartImpLeft([T_B_Wl*T_Wl_Gl], [0.1], [T_Wl_Gl], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        # if self.velma_task_executor.velma.waitForEffectorRight() != 0:
        #     exitError(19)
        print "The right tool is now in 'grip' pose"
        rospy.sleep(0.5)

        tableTF = self.velma_task_executor.velma.getTf("B", "szafka_above")
        x = tableTF.p[0]
        y = tableTF.p[1]
        z = tableTF.p[2]
        
        # rot = PyKDL.Rotation.RPY( 1.540, -1.540, 0)
        rot = PyKDL.Rotation.RPY( 1.540, -1.540, 1.540)
        BT = PyKDL.Frame(rot, PyKDL.Vector(x-0.3, y, z+0.18))

        
        print("Move colse over the cabinet")
        tol = 10.0
        self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 200, 2000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")

        BT = PyKDL.Frame(rot, PyKDL.Vector(x, y, z+0.18))

        print("Move hand over the cabinet")
        tol = 10.0
        self.velma_task_executor.setLeftLWRImpedance(1000, 1000, 200, 2000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")



        print("Put can on the table")
        BT = PyKDL.Frame(rot, PyKDL.Vector(x, y, z+0.1))
        tol = 10.0
        self.velma_task_executor.setLeftLWRImpedance(200, 200, 500, 1000, 1000, 1000)
        if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
            raise Exception("Could not move in cartesian impedance mode")
        if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
            if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
                raise Exception("Could not make it to given position")

        

        print("open hand")
        dest_q = [0.0, 0.0, 0.0, 0]
        self.velma_task_executor.velma.moveHandLeft(dest_q, [1,1,1,1], [5000, 5000, 5000, 5000], 1000, hold=True)
        if self.velma_task_executor.velma.waitForHandLeft() != 0:
            raise Exception("Hand error")
        rospy.sleep(1)

        # print("pull back")
        # BT = PyKDL.Frame(rot, PyKDL.Vector(x, y, z+0.3))
        # tol = 10.0
        # if not self.velma_task_executor.velma.moveCartImpLeft([BT], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, 0.5, tol))):
        #     raise Exception("Could not move in cartesian impedance mode")
        # if self.velma_task_executor.velma.waitForEffectorLeft() != 0:
        #     if not self.velma_task_executor.velma.moveCartImpLeftCurrentPos(start_time=0.01):
        #         raise Exception("Could not make it to given position")
        print("Move base")
        self.velma_task_executor.setJointImpedanceMode()
        self.velma_task_executor.moveMobileBase(-0.1, 0.0, 0.0, 5.0)
        print("Hide hands")
        self.velma_task_executor.hideHands()
        print("Set move pose")
        self.velma_task_executor.moveToStartingPosition()


        if self.__shutdown__:
            return 'shutdown'
        return 'ok'
