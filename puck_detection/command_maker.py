#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

def compute_center(bbox):
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2.0
    cy = (y1 + y2) / 2.0
    return np.array([cx, cy])

class PuckCenterNode(Node):
    def __init__(self):
        super().__init__('puck_center_publisher')
        self.reference_point = np.array([976.0, 325.0])
        self.scale = 600

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/puck_detector/bboxes_raw_position',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/puck_nearest_center_cm',
            10
        )

    def listener_callback(self, msg):
        data = msg.data
        if len(data) % 4 != 0:
            self.get_logger().warn("Invalid bounding box data.")
            return

        centers = []
        for i in range(0, len(data), 4):
            bbox = data[i:i+4]
            center = compute_center(bbox)
            centers.append(center)

        if not centers:
            self.get_logger().info("No centers detected.")
            return

        distances = [np.linalg.norm(c - self.reference_point) for c in centers]
        nearest_center = centers[np.argmin(distances)]

        adjusted = (nearest_center - self.reference_point) / self.scale
        # x,に -1 をかける
        adjusted[0] = -adjusted[0]
        result_msg = Float32MultiArray()
        result_msg.data = adjusted.tolist()

        self.publisher.publish(result_msg)
        self.get_logger().info(f"Published puck center (cm): {adjusted.tolist()}")

def main():
    rclpy.init()
    node = PuckCenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
