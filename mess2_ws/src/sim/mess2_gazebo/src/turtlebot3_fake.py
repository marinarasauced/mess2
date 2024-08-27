#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class FakeTurtleBot3VICON(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__('fake_turtlebot3_vicon')

        self.declare_parameter("agent_name", "ugv")
        self.agent_name = self.get_parameter("agent_name").get_parameter_value().string_value

        self.declare_parameter('tx0', 0.0)
        self.declare_parameter('ty0', 0.0)
        self.declare_parameter('tz0', 0.0)
        self.declare_parameter('rx0', 0.0)
        self.declare_parameter('ry0', 0.0)
        self.declare_parameter('rz0', 0.0)
        self.declare_parameter('rw0', 0.0)

        self.init = TransformStamped()
        self.init.transform.translation.x = self.get_parameter('tx0').get_parameter_value().double_value
        self.init.transform.translation.y = self.get_parameter('ty0').get_parameter_value().double_value
        self.init.transform.translation.z = self.get_parameter('tz0').get_parameter_value().double_value
        self.init.transform.rotation.x = self.get_parameter('rx0').get_parameter_value().double_value
        self.init.transform.rotation.y = self.get_parameter('ry0').get_parameter_value().double_value
        self.init.transform.rotation.z = self.get_parameter('rz0').get_parameter_value().double_value
        self.init.transform.rotation.w = self.get_parameter('rw0').get_parameter_value().double_value

        self.pub_vicon = self.create_publisher(TransformStamped, f'/vicon/{self.agent_name}/{self.agent_name}', 10)
        self.sub_odom  = self.create_subscription(Odometry, 'odom', self.callback_odom, 10)


    def callback_odom(self, msg):
        """
        
        """
        data = TransformStamped()
        data.header.stamp = self.get_clock().now().to_msg()
        data.transform.translation.x = msg.pose.pose.position.x + self.init.transform.translation.x
        data.transform.translation.y = msg.pose.pose.position.y + self.init.transform.translation.y
        data.transform.translation.z = msg.pose.pose.position.z + self.init.transform.translation.z
        data.transform.rotation.x = msg.pose.pose.orientation.x + self.init.transform.rotation.x
        data.transform.rotation.y = msg.pose.pose.orientation.y + self.init.transform.rotation.y
        data.transform.rotation.z = msg.pose.pose.orientation.z + self.init.transform.rotation.z
        data.transform.rotation.w = msg.pose.pose.orientation.w + self.init.transform.rotation.w
        self.pub_vicon.publish(data)


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = FakeTurtleBot3VICON()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
