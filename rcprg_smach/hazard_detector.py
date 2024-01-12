#!/usr/bin/env python3
# encoding: utf8

import rospy
from tiago_msgs.msg import HazardObject 

class HazardDetector:
	def __init__(self):
		subs = rospy.Subscriber('/hazard_objects', HazardObject, self.callback)
		self._last_id = -1
		self.new_hazard = False
		self._hazard_object = None
		self._initialisation_time = rospy.Time.now()

	def callback(self, data):
		self._hazard_object = data

	def check_hazard(self):
		if self._hazard_object != None:
			if self._last_id != self._hazard_object.id and self._initialisation_time < self._hazard_object.header.stamp:
				self._last_id = self._hazard_object.id
				return True, self._hazard_object.object
		
		return False, None
