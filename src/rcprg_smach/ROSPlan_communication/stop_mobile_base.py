import os, rospy
from time import sleep
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped

def send_velocity():
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    move_cmd.linear.x = 0.0
    move_cmd.linear.y = 0.0
    move_cmd.linear.z = 0.0
    move_cmd.angular.x = 0.0
    move_cmd.angular.y = 0.0
    move_cmd.angular.z = 0.0
    t = 0
    while pub.get_num_connections() < 1:
        pass

    pub.publish(move_cmd)
    print("Publishing vel=0.")
    print("Publishing vel=0 completed.")
