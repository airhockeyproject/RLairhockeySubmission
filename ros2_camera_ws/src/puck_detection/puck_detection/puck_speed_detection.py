#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time


class PuckCenterSpeedNode(Node):
    def __init__(self):
        super().__init__('puck_center_speed_publisher')
        # 座標を入れとくリスト
        self.centers_x=[]
        self.centers_y=[]



        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/puck_nearest_center_cm',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/puck_center_speed',
            10
        )

    def listener_callback(self, msg):
        # フレームレート30で割る。これはtopicのhzで求めた。
        data = msg.data
        x_adj=data[0]
        y_adj=data[1]

        centers_x.append(x_adj)
        centers_y.append(y_adj)

        if len(self.centers_x)<2:
            return

        distance_x=centers_x[-1]-centers_x[-2]
        distance_y=centers_y[-1]-centers_y[-2]

        speed_x=distance_x*30
        speed_y=distance_y*30

        publish_list=[speed_x,speed_y]

        result_msg = Float32MultiArray()
        result_msg.data = publish_list

        self.publisher.publish(result_msg)
        self.get_logger().info(f"Published puck center spped: {publish_list}")

def main():
    rclpy.init()
    node = PuckCenterSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()