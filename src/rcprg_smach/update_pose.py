#!/usr/bin/env python
# encoding: utf8

import math
import rospy

from move_base_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose
import tf
from visualization_msgs.msg import Marker

import task_manager

from tasker_msgs.msg import RobotResource, ScheduleParams
from nav_msgs.srv import *
from nav_msgs.msg import *


import rcprg_kb.places_xml as kb_p
from rcprg_smach.task_manager import PoseDescription

def makePose(x, y, theta):
    q = quaternion_from_euler(0, 0, theta)
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.orientation.x = q[0]
    result.orientation.y = q[1]
    result.orientation.z = q[2]
    result.orientation.w = q[3]
    return result

def setMarker(action, publisher,marker_name, x,y,ox,oy,oz,ow):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = marker_name+"_pose"
    marker.id = 0
    marker.type = Marker.ARROW
    if action == 'add':
        marker.action = Marker.ADD
    else:
        marker.action = Marker.MODIFY
 
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.x = ox
    marker.pose.orientation.y = oy
    marker.pose.orientation.z = oz
    marker.pose.orientation.w = ow
    marker.scale.x = 1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()

    name_marker = Marker()
    name_marker.header.frame_id = "/map"
    name_marker.header.stamp = rospy.Time.now()
    name_marker.ns = marker_name+"_name"
    name_marker.id = 0
    name_marker.type = Marker.TEXT_VIEW_FACING
    if action == 'add':
        name_marker.action = Marker.ADD
    else:
        name_marker.action = Marker.MODIFY

    name_marker.pose.position.x = x
    name_marker.pose.position.y = y
    name_marker.pose.position.z = 2
    name_marker.pose.orientation.x = 0.0
    name_marker.pose.orientation.y = 0.0
    name_marker.pose.orientation.z = 0.0
    name_marker.pose.orientation.w = 1.0

    name_marker.text = "KB_"+marker_name

    name_marker.scale.z = 0.3

    name_marker.color.r = 0.0
    name_marker.color.g = 1.0
    name_marker.color.b = 0.0
    name_marker.color.a = 1.0

    publisher.publish(marker)
    publisher.publish(name_marker)

class UpdatePose():
    def __init__(self, sim_mode, kb_places, pose_id):
        self.sim_mode = sim_mode
        self.kb_places = kb_places
        self.listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher(pose_id+"_markers", Marker, queue_size=10)
        self.pose_id = pose_id
        pl = self.kb_places.getPlaceByName(pose_id, "sim")
        pt_dest = pl.getPt()
        norm = pl.getN()
        quat = quaternion_from_euler(0,0,math.atan2(norm[1], norm[0]))
        setMarker('add',self.marker_pub, pose_id, pt_dest[0], pt_dest[1], quat[0],quat[1],quat[2], quat[3])


    def update_pose(self, transform_name):
        if self.sim_mode in ['sim', 'gazebo']:
            mc = self.kb_places.getMapContext('sim')
        elif self.sim_mode == 'real':
            mc = self.kb_places.getMapContext('real')
        else:
            raise Exception('<update_robot_pose> I dont know the map context you use: "' + self.sim_mode + '". I know <sim> and <gazebo> and <real>.')
        if self.sim_mode == 'sim':
            self.pose_id = unicode(self.pose_id)
            mc.updatePointPlace(self.pose_id, self.pose_id, [0, 0], [0, 0])
            return 
        else:
            while not rospy.is_shutdown():
                try:
                    ros_time = rospy.Time()
                    self.listener.waitForTransform('/map', transform_name, ros_time, rospy.Duration(8.0))
                    (trans,rot) = self.listener.lookupTransform('/map', transform_name, ros_time)
                    if len(trans) != 0:
                        break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rate.sleep()
                    continue
            alpha, beta, theta = euler_from_quaternion([rot[0],rot[1],rot[2],rot[3]])
            x = math.cos(theta)
            y = math.sin(theta)
            self.pose_id = unicode(self.pose_id)
            mc.updatePointPlace(self.pose_id, self.pose_id, trans, [x, y])
            setMarker('modify', self.marker_pub, self.pose_id, trans[0], trans[1], rot[0],rot[1],rot[2], rot[3])
