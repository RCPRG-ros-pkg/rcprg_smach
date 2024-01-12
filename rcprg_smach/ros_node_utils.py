#!/usr/bin/env python3
# encoding: utf8

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# class NodeNamesGetter(Node):
#     def __init__(self):
#         super().__init__('node_names_getter')

#     def get_node_names(self, namespace=None):
#         """
#         Get names of active nodes in the ROS2 graph.
#         @param namespace: optional namespace to scope return values by. Namespace must already be resolved.
#         @type  namespace: str
#         @return: list of node names
#         @rtype: [str]
#         """
#         node_names = self.get_node_names_and_namespaces()
#         if namespace:
#             namespace = namespace if namespace.startswith('/') else '/' + namespace
#             filtered_names = [name for name, ns in node_names if ns.startswith(namespace)]
#         else:
#             filtered_names = [name for name, ns in node_names]
#         return list(set(filtered_names))

# def main(args=None):
#     rclpy.init(args=args)
#     node_names_getter = NodeNamesGetter()
#     try:
#         namespace = None  # Set your namespace here if needed
#         node_names = node_names_getter.get_node_names(namespace)
#         print("Active nodes:", node_names)
#     finally:
#         node_names_getter.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node

class ROSNodeUtils(Node):
    def __init__(self):
        super().__init__('ros_node_utils')

    def get_node_names(self, namespace=None):
        node_names = self.get_node_names_and_namespaces()
        if namespace:
            namespace = namespace if namespace.startswith('/') else '/' + namespace
            filtered_names = [name for name, ns in node_names if ns.startswith(namespace)]
        else:
            filtered_names = [name for name, ns in node_names]
        return list(set(filtered_names))

# Outside of the class, you can define a function to be used directly
def get_node_names(namespace=None):
    rclpy.init()
    ros_node_utils = ROSNodeUtils()
    try:
        names = ros_node_utils.get_node_names(namespace)
    finally:
        ros_node_utils.destroy_node()
        rclpy.shutdown()
    return names
