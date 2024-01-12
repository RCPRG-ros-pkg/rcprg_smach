#!/usr/bin/env python3
# encoding: utf8

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from rclpy.clock import Clock

class Nav2Client(Node):

    def __init__(self):
        super().__init__('nav2_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_status = None  # Variable to hold current goal status

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        # Also, keep track of the goal handle to check the status later
        self._goal_handle = goal_handle

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal was canceled')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info('Goal was aborted')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        # Update current goal status
        self.current_goal_status = status

    def feedback_callback(self, feedback_msg):
        # This callback is for getting feedback
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def cancel_goal(self):
        self._goal_handle.cancel_goal_async()

    def cancel_all_goals(self):
        self._action_client.cancel_all_goals_async()


    # Method to get the current status
    def get_current_goal_status(self):
        if self._goal_handle:
            return self._goal_handle.get_status()
        else:
            return None

    def get_clock(self):
        return Clock()

def main(args=None):
    rclpy.init(args=args)
    nav2_client = Nav2Client()

    # Create a PoseStamped object with the goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0

    nav2_client.send_goal(goal_pose)

    # Spin in a separate thread or perform work and query the status periodically
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nav2_client)
    try:
        executor_thread = threading.Thread(target=executor.spin)
        executor_thread.start()

        # Query the current goal status at the desired rate
        while rclpy.ok():
            rclpy.spin_once(nav2_client, timeout_sec=1)
            current_status = nav2_client.get_current_goal_status()
            if current_status is not None:
                nav2_client.get_logger().info(f'Current goal status: {current_status}')
            time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        nav2_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()