import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/puck_detector/image',
            self.image_callback,
            10)
        self.subscription  # prevent unused warning

        # ArUco辞書と検出器の初期化
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def image_callback(self, msg: Image):
        # ROS画像メッセージをOpenCV画像に変換
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'画像変換失敗: {e}')
            return

        # ArUcoマーカー検出
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        if ids is None or len(ids) == 0:
            self.get_logger().info('マーカーは検出されませんでした。')
            return

        # 各マーカーのIDと中心座標を計算しログ出力
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            # marker_cornersは(4,2)の四角の4点
            center = np.mean(marker_corners[0], axis=0)  # (x,y)
            x, y = int(center[0]), int(center[1])
            self.get_logger().info(f'ID={marker_id} のマーカー中心: ({x}, {y})')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
