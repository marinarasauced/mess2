#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from mess2_msgs.msg import (
    Vertex,
    VertexArray,
    Edge,
    EdgeArray,
)

from cv_bridge import CvBridge
import cv2
import numpy as np


class AlgorithmThreatAnnotator(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("threat_annotator")

        self.declare_parameter("image_resolution", 3001)
        self.declare_parameter("x_scaled_min", -3.0)
        self.declare_parameter("x_scaled_max", 3.0)
        self.declare_parameter("y_scaled_min", -3.0)
        self.declare_parameter("y_scaled_max", 3.0)

        self._image_resolution = self.get_parameter("image_resolution").get_parameter_value().integer_value
        self.x_scaled_min = self.get_parameter("x_scaled_min").get_parameter_value().double_value
        self.x_scaled_max = self.get_parameter("x_scaled_max").get_parameter_value().double_value
        self.y_scaled_min = self.get_parameter("y_scaled_min").get_parameter_value().double_value
        self.y_scaled_max = self.get_parameter("y_scaled_max").get_parameter_value().double_value

        self.x_scale = self._image_resolution / (self.x_scaled_max - self.x_scaled_min)
        self.y_scale = self._image_resolution / (self.y_scaled_max - self.y_scaled_min)

        self.bridge = CvBridge()
        self._image_cv2 = None
        self._image_ros2 = None
        self._image_subscription = self.create_subscription(
            Image,
            "visualizer/threat/raw",
            self._image_callback,
            10
        )

        self._image_publisher = self.create_publisher(
            Image,
            "visualizer/threat/annotated",
            10
        )

        self._vertices_subscription = self.create_subscription(
            VertexArray,
            "visualizer/threat/vertices",
            self._vertices_callback,
            10
        )

    
    def _image_publish(self):
        """
        
        """
        self._image_publisher.publish(self._image_ros2)


    def _image_callback(self, msg: Image):
        """
        
        """
        self._image_cv2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._image_ros2_raw = msg
        self._image_ros2 = msg

        self.get_logger().info("plotting threat image")
        self._image_publish()

        # self.get_logger().info("plotting test vertex")
        # self.draw_vertex(center=np.array([0.0, 0.0]))

        self.destroy_subscription(self._image_subscription)


    def _vertices_callback(self, msg: VertexArray):
        """
        
        """
        self.get_logger().info("received")
        self.draw_vertices(msg.vertices)

    
    def convert_meters_to_pixels(self, x_meters, y_meters):
        """
        
        """
        x_pixels = int((x_meters - self.x_scaled_min) * self.x_scale)
        y_pixels = int((y_meters - self.y_scaled_min) * self.y_scale)
        return x_pixels, y_pixels
    

    def prepare_vertex_for_drawing(self, vertex, radius):
        """
        
        """
        x_center = vertex.position.x
        y_center = vertex.position.y
        x_pixels, y_pixels = self.convert_meters_to_pixels(x_center, y_center)
        height, width, _ = self._image_cv2.shape
        x_pixels = np.clip(x_pixels, radius, width - radius - 1)
        y_pixels = np.clip(y_pixels, radius, height - radius - 1)
        return x_pixels, y_pixels


    def draw_vertex(self, vertex, radius=6, color=(0, 0, 0), thickness=-1):
        """
        
        """
        x_pixels, y_pixels = self.prepare_vertex_for_drawing(vertex, radius)
        cv2.circle(self._image_cv2, (x_pixels, y_pixels), radius, color, thickness)
        self._image_ros2 = self.bridge.cv2_to_imgmsg(self._image_cv2, encoding="bgr8")
        self._image_publish()


    def draw_vertices(self, vertices, radius, color=(0, 0, 0), thickness=-1):
        """
        
        """
        for vertex in vertices:
            x_pixels, y_pixels = self.prepare_vertex_for_drawing(vertex, radius)
            cv2.circle(self._image_cv2, (x_pixels, y_pixels), radius, color, thickness)
        self._image_ros2 = self.bridge.cv2_to_imgmsg(self._image_cv2, encoding="bgr8")
        self._image_publish()







def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = AlgorithmThreatAnnotator()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
