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

from ament_index_python.packages import get_package_share_directory
from os import path
import re
import subprocess


PRIMATIVE_DATA_TYPES = {"int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "float32", "float64", "bool", "string", "time", "duration", "byte"}


def get_msg_path(pkg_msg, type_msg):
    """
    
    """
    path_topic = path.join(
        get_package_share_directory(pkg_msg),
        "msg",
        f"{type_msg}.msg"
    )

    return path_topic


def get_msg_map(path_msg, output, struct):
    """
    
    """
    file_msg = open(path_msg, "r")
    lines = file_msg.readlines()
    for line in lines:
        line = line.strip()
        if not line.startswith("#") and line:
            parts = line.split()
            name_field = parts[1]
            type_field = parts[0]

            is_array = re.match(r"(.+?)\[(\d*)\]", type_field)
            is_pkg_diff = type_field.__contains__("/")
            

            if is_array:
                type_base = is_array.group(1)
                size_array = is_array.group(2)
                size_array = int(size_array) if size_array else "unspecified"
                type_field = type_base
            
            is_primative = type_field in PRIMATIVE_DATA_TYPES

            if is_primative:
                output.append(f"{struct}.{name_field}")
            else:
                if (type_field.__contains__("/")):
                    type_field = type_field.split("/")
                    pkg_msg = type_field[0]
                    type_msg = type_field[1]
                    path_msg_ = get_msg_path(pkg_msg, type_msg)
                    get_msg_map(path_msg_, output, struct)
                else:
                    print(path_msg)
                    pkg_msg = path_msg.split("/")[-3]
                    type_msg = type_field
                    path_msg_ = get_msg_path(pkg_msg, type_msg)
                    get_msg_map(path_msg_, output, struct)
    
    return output


def get_topic_map(pkg_topic, type_topic, struct="msg"):
    """
    
    """
    path_topic = get_msg_path(pkg_topic, type_topic)
    map_topic = get_msg_map(path_topic, [], struct)

    return map_topic


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

            self.map_topic = None
            self.type_topic = None
            self.set_topic_info()

        
        def set_topic_info(self):
            """
            
            """
            command = ["ros2", "topic", "type", self.name_topic]
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            output = result.stdout.strip().split("/")
            
            self.map_topic = get_topic_map(output[0], output[2])
            self.type_topic = output[2]
            

            


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
        self.get_logger().info(topic.type_topic)
        print(topic.map_topic)


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
