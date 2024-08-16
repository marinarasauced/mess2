#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

from mess2_msgs.action import UAVCalibrate

import os
import sys

MODULES_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../"))
sys.path.append(MODULES_PATH)

from modules.utils import get_vicon_topic # type: ignore


class CalibrationServer(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("uav_calibration_server")

        self.declare_parameter("agent_name", "agent")
        self.agent_name_ = self.get_parameter("agent_name").get_parameter_value().string_value
        self.vicon_topic_ = get_vicon_topic(self.agent_name_)

        self.global_ = TransformStamped()
        print("done0")
        self._vicon_subscription = self.create_subscription(
            TransformStamped,
            self.vicon_topic_,
            10,
            self.callback_vicon
        )
        print("done1")
        self._calibration_server = ActionServer(
            self,
            UAVCalibrate,
            "calibrate_uav",
            self.callback_calibrate
        )
        print("done2")

    
    def callback_vicon(self, msg: TransformStamped):
        """
        
        """
        self.global_ = msg
    

    def callback_calibrate(self, goal_handle):
        """
        
        """
        self.get_logger().info("executing calibration")
        result = UAVCalibrate.Result()
        return result
        


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CalibrationServer()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
