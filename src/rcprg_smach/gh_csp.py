#!/usr/bin/env python
# encoding: utf8

import math
import rospy

from move_base_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
import tf

import task_manager

from tasker_msgs.msg import RobotResource, ScheduleParams
from tasker_msgs.srv import SuspendConditions, CostConditions, SuspendConditionsResponse, CostConditionsResponse
from nav_msgs.srv import *
from nav_msgs.msg import *
import std_srvs.srv as std_srvs


import rcprg_kb.places_xml as kb_p
from rcprg_smach.task_manager import PoseDescription

def makePose(x, y, theta):
    q = quaternion_from_euler(0, 0, theta)
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.orientation.w = q[3]
    result.orientation.x = q[0]
    result.orientation.y = q[1]
    result.orientation.z = q[2]
    return result

class GH_csp():
    def __init__(self, human_name):
        self.human_name = human_name
        if self.human_name == "Tomek":
            # walk factor for human 0
            self.factor_walk = 0.65#0.15
            # sit factor for human 0
            self.factor_sit = 5#0.3
            # stand factor for human 0
            self.factor_stand = 1#0.15
        else:
            # walk factor for human 1
            self.factor_walk = 1#0.2
            # sit factor for human 1
            self.factor_sit = 1#0.3
            # stand factor for human 1
            self.factor_stand = 1#0.2
            self.listener = tf.TransformListener()
    def get_robot_pose(self, sim_mode, kb_places, robot_name, start_place=None):
        if sim_mode == 'real':
            mc_name = 'real'
        elif sim_mode in ['gazebo', 'sim']:
            mc_name = 'sim'
        try:
            robot_name = unicode(robot_name)
            pl = kb_places.getPlaceByName(robot_name, mc_name)
        except:
            print 'CSP calculator place_name is not valid'
            return 'error'

        if pl.getType() == 'point':
            pt_dest = pl.getPt()
            norm = pl.getN()
        elif pl.getType() == 'volumetric':
            if start_place == None:
                return 'is-volumetric'
                #raise Exception("csp calculator: required pose is volumetric type, but no start place is specified.")
            pose_start = self.get_robot_pose(sim_mode, kb_places, start_place)
            pt_start = [pose_start.position.x, pose_start.position.y]
            pt_dest = kb_places.getClosestPointOfPlace(pt_start, pl.getId(), mc_name)
            norm = [0,0]
        else:
            "csp calculator does not recognise the place type"

        return makePose(pt_dest[0], pt_dest[1], math.atan2(norm[1], norm[0]))
    def get_dest_pose(self, sim_mode, kb_places, place_name, pt_start):

        if sim_mode == 'sim':
            pose = makePose(0, 0, 0)
        else:
            if sim_mode == 'real':
                mc_name = 'real'
            elif sim_mode == 'gazebo':
                mc_name = 'sim'

            if pl.getType() == 'point':
                pt_dest = pl.getPt()
                norm = pl.getN()
                print "NORM_0: ", norm[0]
                print "NORM_1: ", norm[1]
                angle_dest = -math.atan2(norm[1], norm[0])
                pt = pt_dest
                pt_dest = (pt_dest[0]+norm[0], pt_dest[1]+norm[1])
                print 'CSP calculator place type: point'
                print 'pt: {}, pt_dest: {}, norm: {}, angle_dest: {}'.format(pt, pt_dest, norm, angle_dest)
            elif pl.getType() == 'volumetric':
                pt_dest = self.kb_places.getClosestPointOfPlace(pt_start, pl.getId(), mc_name)
                angle_dest = 0.0
            else:
                raise Exception('Unknown place type: "' + pl.getType() + '"')
            pose = makePose(pt_dest[0], pt_dest[1], angle_dest)
        return pose
    def get_cost_conditions_gh(self, cost_request, human_pose, human_name):
        human_posture = rospy.get_param(human_name+"/actor_posture", "stand")
        cost_response = CostConditionsResponse()
        ##
        # plan to get cost from final state given in request to the state required by this DA 
        ##
        rospy.wait_for_service('/move_base/GlobalPlanner/make_plan')
        path_client = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)
        req_path = GetPlanRequest()
        ros_time = rospy.Time()
        trans = []

        path_distance = 0
        req_path.start.header.frame_id = "map"
        req_path.start.pose.position = cost_request.final_resource_state.robot_position
        req_path.start.pose.orientation.x = 0
        req_path.start.pose.orientation.y = 0
        req_path.start.pose.orientation.z = 0
        req_path.start.pose.orientation.w = 0
        req_path.goal.header.frame_id = "map"
        req_path.goal.pose = human_pose
        resp = path_client(req_path)
        while len(resp.plan.poses) < 2:
            print "++++++++++++++++++++++++++++++++++++++"
            print "CANNOT CALCULATE COST CONDITIONS <",human_name,"> "
            print "++++++++++++++++++++++++++++++++++++++"
            rospy.wait_for_service('/move_base/clear_costmaps')
            #try:
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.Empty)
            clear_costmaps()
            resp = path_client(req_path)
            rospy.sleep(1)
        # print "PATH LENGTH: ",len(resp.plan.poses)
        path_distance_approach = 0
        if len(resp.plan.poses) > 2:
            for i in range(len(resp.plan.poses)-1):
                # print "POSE: ",resp.plan.poses[i]
                path_distance_approach += math.sqrt(math.pow((resp.plan.poses[i+1].pose.position.x - resp.plan.poses[i].pose.position.x),2) + pow((resp.plan.poses[i+1].pose.position.y - resp.plan.poses[i].pose.position.y), 2))
        if human_posture == "walk":
            cost_per_sec = self.factor_walk
        if human_posture == "stand":
            cost_per_sec = self.factor_stand
        if human_posture == "sit":
            cost_per_sec = self.factor_sit
        if human_posture == "ZERO":
            cost_per_sec = 0

        cost_response.cost_to_complete = path_distance_approach * cost_per_sec
        return cost_response 

    def get_suspend_conditions_gh(self, suspend_request, greeted, human_pose):

        suspend_response = SuspendConditionsResponse()
        if not greeted:
            suspend_response.cost_per_sec = self.factor_stand
        else:
            suspend_response.cost_per_sec = self.factor_sit
        ##
        # plan to get cost from final state given by a candidate for interrupting task to the state required by this DA 
        ##
        rospy.wait_for_service('/move_base/GlobalPlanner/make_plan')
        path_client = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)
        req_path = GetPlanRequest()
        ros_time = rospy.Time()
        trans = []
        rate = rospy.Rate(5.0)
        path_distance = 0
        req_path.start.header.frame_id = "map"
        req_path.start.pose.position = suspend_request.final_resource_state.robot_position
        req_path.start.pose.orientation.x = 0
        req_path.start.pose.orientation.y = 0
        req_path.start.pose.orientation.z = 0
        req_path.start.pose.orientation.w = 0
        req_path.goal.header.frame_id = "map"
        req_path.goal.pose= human_pose
        resp = path_client(req_path)
        while len(resp.plan.poses) < 2:
            print "++++++++++++++++++++++++++++++++++++++"
            print "CANNOT CALCULATE SUSPEND CONDITIONS <",self.human_name,"> "
            print "++++++++++++++++++++++++++++++++++++++"
            rospy.wait_for_service('/move_base/clear_costmaps')
            #try:
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.Empty)
            clear_costmaps()
            resp = path_client(req_path)
            rospy.sleep(1)
        # print "PATH LENGTH: ",len(resp.plan.poses)
        path_distance_approach = 0
        if len(resp.plan.poses) > 2:
            for i in range(len(resp.plan.poses)-1):
                # print "POSE: ",resp.plan.poses[i]
                path_distance_approach += math.sqrt(math.pow((resp.plan.poses[i+1].pose.position.x - resp.plan.poses[i].pose.position.x),2) + pow((resp.plan.poses[i+1].pose.position.y - resp.plan.poses[i].pose.position.y), 2))
        
        #######
        ####
        ##
        ##   TO DO : skalowanie kosztu dla ptf_cost_condition i ptf_suspend_condition
        ####
        #######

        suspend_response.cost_to_resume = path_distance_approach * suspend_response.cost_per_sec
        return suspend_response 


    def get_schedule_params_gh(self, robot_pose, human_pose, destination_pose, human_name):
        isinstance(human_pose,Pose)
        human_posture = rospy.get_param(human_name+"/actor_posture", "stand")
        sp = ScheduleParams()
        print "CALC COST"
        print "H_POSE: ", human_pose
        print "R_POSE: ", robot_pose

        rospy.wait_for_service('/move_base/GlobalPlanner/make_plan')
        path_client = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)
        req_path = GetPlanRequest()
        ros_time = rospy.Time()
        self.robot_trans = []
        human_trans = []
        robot_rot = []
        human_rot = []
        rate = rospy.Rate(5.0)

        path_distance = 0
        req_path.start.header.frame_id = "map"
        req_path.start.pose = robot_pose
        req_path.goal.header.frame_id = "map"
        req_path.goal.pose = human_pose
        resp = path_client(req_path)
        while len(resp.plan.poses) < 2:
            print "++++++++++++++++++++++++++++++++++++++"
            print "CANNOT CALCULATE APPROACH PATH <",human_name,"> "
            print "++++++++++++++++++++++++++++++++++++++"
            print resp
            rospy.wait_for_service('/move_base/clear_costmaps')
            #try:
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.Empty)
            clear_costmaps()
            resp = path_client(req_path)
            rospy.sleep(1)
        # print "PATH LENGTH: ",len(resp.plan.poses)
        path_distance_approach = 0
        if len(resp.plan.poses) > 2:
            for i in range(len(resp.plan.poses)-1):
                # print "POSE: ",resp.plan.poses[i]
                path_distance_approach += math.sqrt(math.pow((resp.plan.poses[i+1].pose.position.x - resp.plan.poses[i].pose.position.x),2) + pow((resp.plan.poses[i+1].pose.position.y - resp.plan.poses[i].pose.position.y), 2))
        
        req_path.start.header.frame_id = "map"
        req_path.start.pose = human_pose
        req_path.goal.header.frame_id = "map"
        req_path.goal.pose = destination_pose
        resp = path_client(req_path)
        while len(resp.plan.poses) < 2:
            print "++++++++++++++++++++++++++++++++++++++"
            print "CANNOT CALCULATE GUIDE PATH <",human_name,"> "
            print "++++++++++++++++++++++++++++++++++++++"
            rospy.wait_for_service('/move_base/clear_costmaps')
            #try:
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.Empty)
            clear_costmaps()
            resp = path_client(req_path)
            rospy.sleep(1)
        # print "++++++++++++++++++++++++++++++++++++++"
        # print "PATH LENGTH TO:",human_name,": ",path_distance_approach
        # print "++++++++++++++++++++++++++++++++++++++"
        path_distance_guide = 0
        if len(resp.plan.poses) > 2:
            for i in range(len(resp.plan.poses)-1):
                # print "POSE: ",resp.plan.poses[i]
                path_distance_guide += math.sqrt(math.pow((resp.plan.poses[i+1].pose.position.x - resp.plan.poses[i].pose.position.x),2) + pow((resp.plan.poses[i+1].pose.position.y - resp.plan.poses[i].pose.position.y), 2))
              
        # print "++++++++++++++++++++++++++++++++++++++"
        # print "PATH LENGTH DEST:",human_name,": ",path_distance_guide
        # print "++++++++++++++++++++++++++++++++++++++"
        if human_posture == "walk":
            sp.cost_per_sec = self.factor_walk
            cost = path_distance_guide * self.factor_walk
        elif human_posture == "stand":
            sp.cost_per_sec = self.factor_stand
            cost = path_distance_approach*self.factor_stand + path_distance_guide * self.factor_walk
        elif human_posture == "sit":
            sp.cost_per_sec = self.factor_sit
            cost = path_distance_approach*self.factor_sit + path_distance_guide * self.factor_walk
        elif human_posture == "ZERO":
            sp.cost_per_sec = 0        
        elif human_posture == "fall":
            sp.cost_per_sec = self.factor_stand
            cost = path_distance_approach*self.factor_stand + path_distance_guide * self.factor_walk
        else:
            sp.cost_per_sec = self.factor_stand
            cost = path_distance_approach*self.factor_stand + path_distance_guide * self.factor_walk
        print "COST: ", cost
        sp.cost = cost
        sp.completion_time = path_distance_approach + path_distance_guide
        sp.final_resource_state.robot_position = req_path.goal.pose.position
        return sp

    def get_schedule_params_hf(self, robot_pose, human_pose):
        sp = ScheduleParams()
        print "CALC COST"

        path_client = rospy.ServiceProxy('/move_base/GlobalPlanner/make_plan', GetPlan)
        req_path = GetPlanRequest()
        ros_time = rospy.Time()
        self.robot_trans = []
        human_trans = []
        robot_rot = []
        human_rot = []
        rate = rospy.Rate(5.0)

        path_distance = 0
        req_path.start.header.frame_id = "map"
        req_path.start.pose = robot_pose
        req_path.goal.header.frame_id = "map"
        req_path.goal.pose = human_pose
        resp = path_client(req_path)
        # print "PATH LENGTH: ",len(resp.plan.poses)
        path_distance_approach = 0
        if len(resp.plan.poses) > 2:
            for i in range(len(resp.plan.poses)-1):
                # print "POSE: ",resp.plan.poses[i]
                path_distance_approach += math.sqrt(math.pow((resp.plan.poses[i+1].pose.position.x - resp.plan.poses[i].pose.position.x),2) + pow((resp.plan.poses[i+1].pose.position.y - resp.plan.poses[i].pose.position.y), 2))
        sp.cost = path_distance_approach
        sp.completion_time = 1
        sp.final_resource_state.robot_position = req_path.goal.pose.position
        return sp

