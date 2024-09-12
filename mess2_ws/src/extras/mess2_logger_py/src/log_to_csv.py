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
import csv
import datetime
from os import listdir, makedirs, path, unlink
import re
import shutil
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
                    get_msg_map(path_msg_, output, f"{struct}.{name_field}")
                else:
                    print(path_msg)
                    pkg_msg = path_msg.split("/")[-3]
                    type_msg = type_field
                    path_msg_ = get_msg_path(pkg_msg, type_msg)
                    get_msg_map(path_msg_, output, f"{struct}.{name_field}")
    
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

        self.declare_parameter("dir_logs", "/home/mess2/mess2/logs/2024_09_12/testing")
        self.declare_parameter("name_experiment", "demo1")
        self.declare_parameter("topics", [""])

        dir_logs = self.get_parameter("dir_logs").get_parameter_value().string_value
        name_experiment = self.get_parameter("name_experiment").get_parameter_value().string_value
        topics = self.get_parameter("topics").get_parameter_value().string_array_value

        if not path.exists(dir_logs):
            makedirs(dir_logs)

        topic1 = self.Topic(topics[0], dir_logs)
        self.set_topic_subscription(topic1)
        self.print_topic_info(topic1)


    class Topic():
        """
        
        """
        def __init__(self, topic, dir_logs):
            """
            
            """
            self.name_topic = topic

            self.log_topic = None
            self.map_topic = None
            self.type_topic = None
            self.set_topic_info(dir_logs)

            if not path.exists(self.log_topic):
                with open(self.log_topic, "w", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(self.map_topic)

        
        def set_topic_info(self, dir_logs):
            """
            
            """
            command = ["ros2", "topic", "type", self.name_topic]
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            output = result.stdout.strip().split("/")
            
            self.map_topic = get_topic_map(output[0], output[2])
            self.type_topic = output[2]

            file_topic = self.name_topic[1:].replace("/", "_") + ".csv"
            self.log_topic = path.join(dir_logs, file_topic)
            if not path.exists(self.log_topic):
                makedirs(self.log_topic)
            for file in listdir(dir_logs):
                path_file = path.join(dir_logs, file)
                if path.isfile(path_file) or path.islink(path_file):
                    unlink(path_file)
                elif path.isdir(path_file):
                    shutil.rmtree(path_file)


        def callback(self, msg):
            """
            
            """
            list = []
            for map_field in self.map_topic:
                try:
                    data_field = str(eval(f"{map_field}"))
                    if data_field.startswith("()") and data_field.endswith(")"):
                        field_data = field_data.replace(",", "")
                    list.append(data_field)
                except Exception as e:
                    list.append("")
            
            with open(self.log_topic, "a", newline="") as file:
                writer = csv.writer(file, delimiter="\t")
                writer.writerow(list)
            list.clear()


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
        self.get_logger().info(f"{topic.map_topic}")


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
