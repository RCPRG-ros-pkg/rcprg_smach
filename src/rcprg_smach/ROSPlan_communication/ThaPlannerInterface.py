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
import shutil
from kb_communication.query_kb_state import update_kb, update_goal
from os.path import dirname, join

class ThPlInterface:
    def __init__(self, da_type, da_id, args, executable_tasks, cfg_actions, tf, planner_type="ENHSP", metric="battery-capacity"):
        self.rcprg_path = dirname(dirname(dirname(dirname(os.path.abspath(__file__)))))
        self.src_path = dirname(dirname(dirname(dirname(dirname(os.path.abspath(__file__))))))
        self.pddl_files_dir = join(self.rcprg_path, "ROSPlan/rosplan_plans/src/pddl")
        self.launch_planner_path = "roslaunch rosplan_plans launch_planner_interface.launch \
        domain_path:=DOMAIN problem_path:=PROBLEM task_problem_path:=TASK_PROBLEM data_path:=DATA"
        self.launch_planner_for_cost_and_duration_path = "roslaunch rosplan_plans \
        launch_rosplan_for_cost_and_duration.launch domain_path:=DOMAIN problem_path:=PROBLEM \
        task_problem_path:=TASK_PROBLEM data_path:=DATA kb_name:=KB_NAME pg_name:=PG_NAME pl_name:=PL_NAME \
        problem_topic:=PRB_TOPIC planner_output_topic:=PL_OUTPUT_TOPIC"
        self.enhsp_exec_path = join(self.src_path, "utils/ENHSP-Public/enhsp-dist/enhsp.jar")
        self.enhsp_planner_path = "timeout 10 java -jar" + self.enhsp_exec_path + " -o DOMAIN -f PROBLEM"
        self.smt_exec_path = join(self.src_path, "utils/SMTPlan/SMTPlan/build/SMTPlan")
        self.smt_planner_path_debug = "timeout 1000 " + self.smt_exec_path +  " DOMAIN PROBLEM -l 4 -u 10 -s 2 -d"
        self.smt_planner_path = "timeout 1000 " + self.smt_exec_path +  " DOMAIN PROBLEM -l 4 -u 10 -s 2"
        self.executable_tasks = executable_tasks
        self.cfg_actions = cfg_actions
        self.tf = tf
        self.planner_type = planner_type
        self.planner_debug_options_on = False
        self.domain_path = ''
        self.domain_smt_path = ''
        self.problem_path = ''
        self.task_problem_path = ''
        self.data_path = ''
        self.plan_path = ''
        self.unprocessed_plan_path = ''
        self.da_type = da_type
        self.duration = 0.0
        self.cost = 0.0
        self.metric = metric
        self.args = args
        self.goal = ''
        self.goals = []
        # self.da_id = da_id
        for i in range(len(self.args)):
            if self.args[i] == "da_id":
                self.da_id = int(self.args[i+1])
                break
        self.call_problem_generation_cmd = "rosservice call /rosplan_problem_interface_"+str(self.da_id)+"/problem_generation_server"
        self.call_planner_cmd = "rosservice call /rosplan_planner_interface_"+str(self.da_id)+"/planning_server"


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
                        if 'unprocessed_plan.pddl' in filepath:
                            self.unprocessed_plan_path = filepath
                self.data_path = subdir
                if self.plan_path == '':
                    self.plan_path = join(self.data_path, 'plan.pddl')
                if self.unprocessed_plan_path == '':
                    self.unprocessed_plan_path = join(self.data_path, 'unprocessed_plan.pddl')
                        
    # RUNNING PLANNER INTERFACE
    def run_planner_for_cost_and_duration(self):
        self.find_pddl_files()
        # REPLACE DOMAIN AND PROBLEM WITH DOMAIN PATH AND PROBLEM PATH
        self.launch_planner_for_cost_and_duration_path = self.launch_planner_for_cost_and_duration_path.replace('DOMAIN', self.domain_path)
        self.launch_planner_for_cost_and_duration_path = self.launch_planner_for_cost_and_duration_path.replace('TASK_PROBLEM', self.task_problem_path)
        self.launch_planner_for_cost_and_duration_path = self.launch_planner_for_cost_and_duration_path.replace('PROBLEM', self.problem_path)
        self.launch_planner_for_cost_and_duration_path = self.launch_planner_for_cost_and_duration_path.replace('DATA', self.data_path)
        self.launch_planner_for_cost_and_duration_path = self.launch_planner_for_cost_and_duration_path.replace('KB_NAME', "rosplan_knowledge_base_"+str(self.da_id))
        self.launch_planner_for_cost_and_duration_path = self.launch_planner_for_cost_and_duration_path.replace('PL_NAME', "rosplan_planner_interface_"+str(self.da_id))
        self.launch_planner_for_cost_and_duration_path = self.launch_planner_for_cost_and_duration_path.replace('PG_NAME', "rosplan_problem_interface_"+str(self.da_id))
        self.launch_planner_for_cost_and_duration_path = self.launch_planner_for_cost_and_duration_path.replace('PRB_TOPIC', "/rosplan_problem_interface_"+str(self.da_id)+"/problem_instance")
        self.launch_planner_for_cost_and_duration_path = self.launch_planner_for_cost_and_duration_path.replace('PL_OUTPUT_TOPIC', "/rosplan_planner_interface_"+str(self.da_id)+"/planner_output")
        # print("EXECUTING PLANNER FOR DYNAMIC AGENT " + self.da_type)
        print("Calling ROSPlan execution")
        subp = subprocess.Popen(shlex.split(self.launch_planner_for_cost_and_duration_path))
        p = psutil.Process(subp.pid)
        print("Waiting for ROSPlan to setup")
        rospy.sleep(2)
        print("Updating KB from MongoDB")
        update_kb(self.da_id, self.da_type, self.executable_tasks, self.args, self.cfg_actions, self.tf)
        rospy.sleep(2)
        print("Updating plan goal, with location given in the task description")
        if self.da_type in self.executable_tasks:
            if self.da_type == "clean":
                for i in range(len(self.args)):
                    if self.args[i] == "m1":
                        self.goals.append(self.args[i+1])
                    if self.args[i] == "m2":
                        self.goals.append(self.args[i+1])
                    if self.args[i] == "m3":
                        self.goals.append(self.args[i+1])
                update_goal(self.goals, self.da_type, self.da_id)
            else:
                for i in range(len(self.args)):
                    if self.args[i] == "miejsce":
                        self.goal = self.args[i+1]
                        break
                update_goal(self.goal, self.da_type, self.da_id)
            
        print("ROSPlan goal updated to " + str(self.goal))
        rospy.sleep(2)
        print("Calling problem generation")
        subp = subprocess.Popen(shlex.split(self.call_problem_generation_cmd))
        p = psutil.Process(subp.pid)
        rospy.sleep(3)
        print("Problem generated")
        print("Calling planner")
        subp = subprocess.Popen(shlex.split(self.call_planner_cmd))
        p = psutil.Process(subp.pid)
        rospy.sleep(3)
        print("Planner called")

    # READ PLAN FILE CONTENT
    # FIND GENERATED PLAN FILE
    def find_plan_file(self):
        for subdir, dirs, files in os.walk(self.pddl_files_dir):
            if self.da_type in subdir:
                for inner_subdir, inner_dirs, inner_files in os.walk(subdir):
                    for inner_file in inner_files:
                        filepath = inner_subdir + os.sep + inner_file
                        # print(filepath)
                        if filepath == 'plan.pddl':
                            self.plan_path = filepath

    # READ GENERATED PLAN FILE
    def read_plan(self, plan_path=None):
        if plan_path is None:
            plan_path = self.plan_path

        if self.planner_type == "SMT" and self.planner_debug_options_on:
            duration_obtained = False
            cost_obtained = False
            pddl_plan = open(plan_path)
            for line in reversed(pddl_plan.readlines()):
                print(line)
                try:
                    if not duration_obtained:
                        self.duration = round(float(line[9:13]), 2)
                        duration_obtained = True
                    if self.metric in line and not cost_obtained:
                        self.cost = round(float(line[line.find("==")+3:line.find("==")+8]), 2)
                        cost_obtained = True
                except:
                    print("Error occured while reading the plan.")
                    self.duration = 0
                    self.cost = 0
                    break
            pddl_plan.close()
        
        if self.planner_type == "SMT" and not self.planner_debug_options_on:
            print("PLAN PATH")
            print(plan_path)
            pddl_plan_len = 0
            with open(plan_path) as pddl_plan:
                try:
                    pddl_plan_len = len(pddl_plan.readlines())
                except:
                    print("Error occured while reading the plan.")
            with open(plan_path) as pddl_plan:
                try:
                    last_line = pddl_plan.readlines()[pddl_plan_len-1]
                    plan_duration = float(last_line[:3])
                    print(plan_duration)
                except:
                    print("Error occured while reading the plan.")

            # Calculating plan cost using coefficients specific for different tasks
            # temporary solution, due to SMTPlan solver error (segmentation fault)
            # Setting some non 0 value for plan duration if it is 0 (same reason as above)
            if plan_duration == 0:
                plan_duration = 3

            self.duration = plan_duration
            if self.da_type in executable_tasks:
                self.cost = 2*plan_duration
            else:
                self.cost = plan_duration

        elif self.planner_type == "ENHSP":
            plan_file = open(self.unprocessed_plan_path, 'r')
            lines = plan_file.readlines()
            for line in lines:
                # CHECK WHETHER LINE CONTAINS INFO ABOUT TASK DURATION
                if "Duration" in line:
                    self.duration = round(float(line[9:]), 2)
            for line in lines:
                # CHECK WHETHER LINE CONTAINS INFO ABOUT TASK COST
                if "Metric" in line and self.duration > 0:
                    self.cost = round(float(line[13:]), 2)
                    self.cost -= self.duration

    def save_plan(self):
        task_sufix = "plan_"+str(self.da_id)+".pddl"
        task_plan_path = join(self.data_path, task_sufix)
        shutil.copyfile(self.plan_path, task_plan_path)
    
    def kill_rosplan_nodes(self):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            if "/rosplan_knowledge_base_"+str(self.da_id) in node:
                os.system("rosnode kill "+ node)
            elif "/rosplan_planner_interface_"+str(self.da_id) in node:
                os.system("rosnode kill "+ node)
            elif "/rosplan_problem_interface_"+str(self.da_id) in node:
                os.system("rosnode kill "+ node)
            print("ROS Info:" +str(node)+ "killed successfully.")

    def exec_plan(self):
        save_cost_to_file = True
        self.run_planner_for_cost_and_duration()
        self.find_plan_file()
        self.kill_rosplan_nodes()
        self.save_plan()
        self.read_plan()
        if save_cost_to_file:
            lines = [str(self.da_id), str(self.da_type), str(self.duration), str(self.cost)]
            with open('/home/mswiech/tiago_public_ws/src/rcprg/tasker/src/TaskER/tasks_costs_durations.txt', 'a') as f:
                for line in lines:
                    print(line)
                    f.write(line)
                    f.write(" ")
                f.write("\n")
        return(self.cost, self.duration)

    def exec_plan_for_exec_da(self):
        # getting da_id to get current execution progress from mongodb
        save_cost_to_file = True
        self.run_planner_for_cost_and_duration()
        self.find_plan_file()
        self.kill_rosplan_nodes()
        self.save_plan()
        self.read_plan()
        if save_cost_to_file:
            lines = [str(self.da_id), str(self.da_type), str(self.duration), str(self.cost)]
            with open('/home/mswiech/tiago_public_ws/src/rcprg/tasker/src/TaskER/tasks_costs_durations.txt', 'a') as f:
                for line in lines:
                    print(line)
                    f.write(line)
                    f.write(" ")
                f.write("\n")
        return(self.cost, self.duration)

    # FUNCTION TO CALL PLANNER ONLY (WITHOUT ROSPlan FRAMEWORK)
    # USED TO GET TASK COST AND DURATION FOR SCHEDULING
    def call_planner(self):
        planner_cmd = ""
        self.find_pddl_files()
        if self.planner_type == "ENHSP":
            planner_cmd = self.enhsp_planner_path
        else:
            # DEBUG OPTIONS STARTED CAUSING SEGMENTATION FAULT ERROR
            # DEPRECATED. TO BE UNCOMMENTED WHEN SOLVED!!!

            # planner_cmd = self.smt_planner_path_debug
            planner_cmd = self.smt_planner_path
            # self.domain_path = self.domain_smt_path
        planner_cmd = planner_cmd.replace('DOMAIN', self.domain_path)
        planner_cmd = planner_cmd.replace('PROBLEM', self.problem_path)
        # SAVE PLAN TO FILE
        planner_cmd = planner_cmd + " > " + self.data_path + "/plan.pddl"

        print("Running: " + planner_cmd)
        subprocess.call(planner_cmd, shell=True)


# th_pl = ThPlInterface("move_new", ["miejsce", "kuchnia"])
# (cost, duration) = th_pl.exec_plan()
# th_pl.call_planner()

# th_pl.call_planner()
# th_pl.kill_rosplan_nodes()
# (cost, duration) = th_pl.exec_plan()


# Running: timeout 400 /home/maciek/tiago_public_ws/src/rcprg/ROSPlan/
# rosplan_planning_system/common/bin/SMTPlan/SMTPlan/build/SMTPlan 
# /home/maciek/tiago_public_ws/src/rcprg/ROSPlan/rosplan_plans/src
# /pddl/move_new_plans/domain_simple.pddl /home/maciek/tiago_public_ws
# /src/rcprg/ROSPlan/rosplan_plans/src/pddl/move_new_plans/task_problem_simple.pddl 
# -l 1 -u 10 -s 1 -d    > /home/maciek/tiago_public_ws/src/rcprg/ROSPlan/rosplan_plans
# /src/pddl/move_new_plans/plan.pddl > /home/maciek/tiago_public_ws/src/rcprg/ROSPlan
# /rosplan_plans/src/pddl/move_new_plans/plan.pddl
