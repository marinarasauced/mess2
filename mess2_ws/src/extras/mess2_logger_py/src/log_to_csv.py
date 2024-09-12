#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import *
from builtin_interfaces.msg import *
from diagnostic_msgs.msg import *
from example_interfaces.msg import *
from flir_camera_msgs.msg import *
from geometry_msgs.msg import *
from geographic_msgs.msg import *
from lifecycle_msgs.msg import *
from map_msgs.msg import *
from mavros_msgs.msg import *
from nav_msgs.msg import *
from pcl_msgs.msg import *
from pendulum_msgs.msg import *
from rosbag2_interfaces.msg import *
from rcl_interfaces.msg import *
from rmw_dds_common.msg import *
from sensor_msgs.msg import *
from shape_msgs.msg import *
from stereo_msgs.msg import *
from statistics_msgs.msg import *
from std_msgs.msg import *
from tf2_msgs.msg import *
from trajectory_msgs.msg import *
from turtlesim.msg import *
from unique_identifier_msgs.msg import *
from visualization_msgs.msg import *

import subprocess





class LogTopicsToCSVs(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("log_to_csvs")

        self.declare_parameter("topics", [""])
        topics = self.get_parameter("topics").get_parameter_value().string_array_value

        topic1 = self.Topic(topics[0])
        self.set_topic_subscription(topic1)
        self.print_topic_info(topic1)

    
    class Topic():
        """
        
        """
        def __init__(self, topic):
            """
            
            """
            self.name_topic = topic
            self.path_topic = None
            self.type_topic = None
            self.set_topic_info()

        
        def set_topic_info(self):
            """
            
            """
            command = ["ros2", "topic", "type", self.name_topic]
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            output = result.stdout.strip()
            
            self.path_topic = output
            self.type_topic = output.split("/")[-1]


        def callback(self, msg):
            """
            
            """
            pass
    
    def set_topic_subscription(self, topic: Topic):
        """
        
        """
        topic.subscription = self.create_subscription(
            eval(topic.type_topic),
            topic.name_topic,
            topic.callback,
            10
        )

        
    def print_topic_info(self, topic: Topic):
        """
        
        """
        self.get_logger().info(topic.name_topic)
        self.get_logger().info(topic.path_topic)
        self.get_logger().info(topic.type_topic)


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = LogTopicsToCSVs()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
