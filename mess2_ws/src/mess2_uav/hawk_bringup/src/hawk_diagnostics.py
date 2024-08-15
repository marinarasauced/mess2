#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode


class HawkDiagnostics(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("hawk_diagnostics")

        self.client_arm_ = self.create_client(CommandBool, "cmd/arming")
        while not self.client_arm_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')

        self.client_mode_ = self.create_client(SetMode, "set_mode")
        while not self.client_mode_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set mode service not available, waiting again...')


    def destroy_node(self):
        """
        Clean up resources before destroying the node and force the Hawk to land before shutdown.
        """
        super().destroy_node()

    
    def set_arm(self, bool):
        """
        
        """
        self.get_logger().info("sending arming request")
        req = CommandBool.Request()
        req.value = bool
        future = self.client_arm_.call_async(req)
        response = future.result()
        self.get_logger().info(f"received arming response: {response.success}")
        return response.success


    def set_mode(self, string):
        """
        
        """
        self.get_logger().info("sending set mode request")
        req = SetMode.Request()
        req.custom_mode = string
        future = self.client_arm_.call_async(req)
        response = future.result()
        self.get_logger().info(f"received set mode response: {response.success}")
        return response.success


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = HawkDiagnostics()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
