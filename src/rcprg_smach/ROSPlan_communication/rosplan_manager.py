from query_kb_state import update_kb_snapshot, get_plan, get_full_plan, remove_goal
from stop_mobile_base import send_velocity
from db_connection import dbConnector
from ROSPlanExecInterface import ROSPlanExecInterface
import os
import rospy

class rosplan_manager:
    def __init__(self, task_id, task_name):
        self.task_id = task_id
        self.task_name = task_name
        self.dbc = dbConnector(task_id)
        self.if_restart = self.check_if_restart()

    def get_plan(self):
        return(get_plan())
    
    def check_if_restart(self):
        pass
        # self.dbc.read_db_restart()        
    
    def remove_goal(self, action_id):
        plan = get_full_plan()
        plan_actions = plan.plan
        print("Plan actions")
        print(plan_actions)
        reached_goal = None
        if self.task_name == "clean" or self.task_name == "move":
            for action in plan_actions:
                if action.action_id == action_id:
                    action_params = action.parameters
                    for param in action_params:
                        print("PARAM")
                        print(param)
                        if param.key == 'end-loc':
                            remove_goal(str(param.value))
        print("GOAL REMOVED")
        


    def before_shutdown(self):
        # sending command for robot to stop
        send_velocity()
        # saving kb state to DB
        update_kb_snapshot(self.task_id)
        print("MongoDB updated.")
        rospy.sleep(2)
        # inserting the info that task has been preempted
        self.dbc.insert_if_restart({'restart': 1})
        # killing action interface manager node
        # self.kill_actinf()
        # sending command for robot to stop
        # killing rosplan nodes
        self.kill_rosplan()
        # print("Deleting task plan file")
        # self.del_task_plan_file()

    def before_finished_shutdown(self):
        # sending command for robot to stop
        send_velocity()
        # deleting MongoDB
        self.dbc.drop_db()
        print("MongoDB dropped.")
        # killing action interface manager node
        # self.kill_actinf()
        # killing rosplan nodes
        self.kill_rosplan()
        print("Deleting task plan file")
        self.del_task_plan_file()

    def del_task_plan_file(self):
        rpl_ei = ROSPlanExecInterface(da_type=self.task_name, da_id = self.task_id)
        rpl_ei.delete_plan_file()


    def kill_actinf(self):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            print(node)
            if node == "/action_interface_manager":
                os.system("rosnode kill "+ node)
                print("ROS Info: " + str(node) + "killed successfully.")

    def kill_rosplan(self):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")

        for node in nodes:
            try:
                # print(node)
                if "/rosplan" in node:
                    os.system("rosnode kill "+ node)
                    print("ROS Info: " + str(node) + " killed successfully.")
            except:
                print("Execption while trying to kill node " + str(node) + ". Exception handled.")
    
    def kill_actinf(self):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        for node in nodes:
            try:
                if "action_interface_manager" in node:
                    os.system("rosnode kill "+ node)
                    print("ROS Info: " + str(node) + " killed successfully.")
            except:
                print("Execption while trying to kill node " + str(node) + ". Exception handled.")







