# -*- coding: utf-8 -*-
import threading
import time
import subprocess
import rospy
import rosnode
import rosservice
import os
import multiprocessing
import psutil
import shlex
from query_kb_state import update_kb_from_db
from os.path import dirname, join

class ROSPlanExecInterface:
    def __init__(self, da_type, da_id, goal="kuchnia", planner_type="ENHSP", metric="battery-capacity"):
        self.rcprg_path = dirname(dirname(dirname(dirname(dirname(os.path.abspath(__file__))))))
        self.src_path = dirname(dirname(dirname(dirname(dirname(dirname(os.path.abspath(__file__)))))))
        self.pddl_files_dir = join(self.rcprg_path, "ROSPlan/rosplan_plans/src/pddl")
        self.launch_full_rosplan_path = "roslaunch rosplan_plans launch_full_rosplan.launch \
        domain_path:=DOMAIN problem_path:=PROBLEM task_problem_path:=TASK_PROBLEM data_path:=DATA"

        self.launch_rosplan_path = "roslaunch rosplan_plans launch_rosplan_smach.launch domain_path:=DOMAIN \
                                problem_path:=PROBLEM task_problem_path:=TASK_PROBLEM data_path:=DATA"
        self.call_parser_cmd = "rosservice call /rosplan_parsing_interface/parse_plan"
        self.call_parser_from_file_cmd = "rosservice call /rosplan_parsing_interface/parse_plan_from_file "
        self.call_dispatcher_cmd = "rosservice call /rosplan_plan_dispatcher/dispatch_plan"
        self.call_problem_generation_cmd = "rosservice call /rosplan_problem_interface/problem_generation_server"
        self.call_planner_cmd = "rosservice call /rosplan_planner_interface/planning_server"
        self.planner_type = planner_type
        self.domain_path = ''
        self.domain_smt_path = ''
        self.problem_path = ''
        self.task_problem_path = ''
        self.data_path = ''
        self.da_type = da_type
        self.da_id = da_id
        self.metric = metric
        self.goal = goal

    # FIND DOMAIN AND PROBLEM FILES 
    # CORRESPONDING PROVIDED TASK TYPE
    def find_pddl_files(self):
        for subdir, dirs, files in os.walk(self.pddl_files_dir):
            if self.da_type in subdir:
                for inner_subdir, inner_dirs, inner_files in os.walk(subdir):
                    for inner_file in inner_files:
                        filepath = inner_subdir + os.sep + inner_file
                        if 'domain_simple.pddl' in filepath:
                            if self.planner_type == "SMT":
                                self.domain_path = filepath                                
                        elif 'domain_ENHSP.pddl' in filepath:
                            if self.planner_type == "ENHSP":
                                self.domain_path = filepath
                        
                        if 'task_problem_simple.pddl' in filepath:
                            if self.planner_type == "SMT":
                                self.task_problem_path = filepath
                        if 'task_problem_ENHSP.pddl' in filepath:
                            if self.planner_type == "ENHSP":
                                self.task_problem_path = filepath

                        if 'problem_simple.pddl' in filepath and "task" not in filepath:
                            if self.planner_type == "SMT":
                                self.problem_path = filepath
                        if 'problem_ENHSP.pddl' in filepath and "task" not in filepath:
                            if self.planner_type == "ENHSP":
                                self.problem_path = filepath
                        
                        if 'plan.pddl' in filepath and 'unprocessed_plan.pddl' not in filepath:
                            self.plan_path = filepath
                        if 'unprocessed_plan.pddl' not in filepath:
                            self.unprocessed_plan_path = filepath
                self.data_path = subdir
                if self.plan_path == '':
                    self.plan_path = join(self.data_path, 'plan.pddl')
                if self.unprocessed_plan_path == '':
                    self.unprocessed_plan_path = join(self.data_path, 'unprocessed_plan.pddl')
    
    # RUNNING PROBLEM GENERATION AND PLANNING INTERFACE
    def run_rosplan_if_restart(self):
        self.find_pddl_files()
        # REPLACE DOMAIN AND PROBLEM WITH DOMAIN PATH AND PROBLEM PATH
        self.launch_full_rosplan_path = self.launch_full_rosplan_path.replace('DOMAIN', self.domain_path)
        self.launch_full_rosplan_path = self.launch_full_rosplan_path.replace('TASK_PROBLEM', self.problem_path)
        self.launch_full_rosplan_path = self.launch_full_rosplan_path.replace('PROBLEM', self.problem_path)
        self.launch_full_rosplan_path = self.launch_full_rosplan_path.replace('DATA', self.data_path)
        print("Calling Planner execution")
        subp = subprocess.Popen(shlex.split(self.launch_full_rosplan_path))
        p = psutil.Process(subp.pid)
        rospy.sleep(2)
        print("Updating KB from MongoDB")
        update_kb_from_db(self.da_id)
        rospy.sleep(2)
        print("Calling problem generation")
        subp = subprocess.Popen(shlex.split(self.call_problem_generation_cmd))
        p = psutil.Process(subp.pid)
        rospy.sleep(2)
        print("Problem generated")
        print("Calling planner")
        subp = subprocess.Popen(shlex.split(self.call_planner_cmd))
        p = psutil.Process(subp.pid)
        rospy.sleep(2)
        print("Planner called")
        print("Calling parser")
        subp = subprocess.Popen(shlex.split(self.call_parser_cmd))
        p = psutil.Process(subp.pid)
        rospy.sleep(2)
        print("Calling dispatcher")
        subp = subprocess.Popen(shlex.split(self.call_dispatcher_cmd))
        p = psutil.Process(subp.pid)


    # RUNNING PARSING AND DISPATCH INTERFACE
    def run_plan_parse_and_dispatch(self):
        self.find_pddl_files()
        # REPLACE DOMAIN AND PROBLEM WITH DOMAIN PATH AND PROBLEM PATH
        self.launch_rosplan_path = self.launch_rosplan_path.replace('DOMAIN', self.domain_path)
        self.launch_rosplan_path = self.launch_rosplan_path.replace('TASK_PROBLEM', self.problem_path)
        self.launch_rosplan_path = self.launch_rosplan_path.replace('PROBLEM', self.problem_path)
        self.launch_rosplan_path = self.launch_rosplan_path.replace('DATA', self.data_path)

        # print("EXECUTING PLANNER FOR DYNAMIC AGENT " + self.da_type)

        subp = subprocess.Popen(shlex.split(self.launch_rosplan_path))
        p = psutil.Process(subp.pid)
        rospy.sleep(3)
        print("Calling Parsing Interface")
        self.call_plan_parsing()
        rospy.sleep(3)
        subp = subprocess.Popen(shlex.split(self.call_parser_from_file_cmd))
        p = psutil.Process(subp.pid)
        rospy.sleep(3)
        print("Calling Dispatch Interface")
        subp = subprocess.Popen(shlex.split(self.call_dispatcher_cmd))
        p = psutil.Process(subp.pid)
    
    # READ TASK PLAN FILE CONTENT
    def call_plan_parsing(self):
        # READING SAVED PLAN FROM FILE (SAVED IN HARMONIZER)
        task_sufix = "plan_"+str(self.da_id)+".pddl"
        task_plan_path = join(self.data_path, task_sufix)
        call_parser_sufix = '"plan_path: ' + "'" + task_plan_path + "'" + '"'
        self.call_parser_from_file_cmd = self.call_parser_from_file_cmd + call_parser_sufix
    
    def delete_plan_file(self):
        # DELETING OF FILE WITH TASK PLAN (AFTER TASK IS FINISHED/INTERRUPTED)
        self.find_pddl_files()
        task_sufix = "plan_"+str(self.da_id)+".pddl"
        task_plan_path = join(self.data_path, task_sufix)
        try:
            os.remove(task_plan_path)
            print("Successfully deleted the plan file of task of id: " + str(self.da_id))
        except:
            print("Unable to delete the task's plan file.")

    def kill_rosplan_nodes(self):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            if "/rosplan_knowledge_base" in node:
                os.system("rosnode kill "+ node)
            elif "/rosplan_planner_interface" in node:
                os.system("rosnode kill "+ node)
            elif "/rosplan_problem_interface" in node:
                os.system("rosnode kill "+ node)
