import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray  # バウンディングボックス座標用
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class PuckDetectorNode(Node):
    def __init__(self):
        super().__init__('puck_detector_node')
        self.bridge = CvBridge()

        # YOLOv8モデルロード（パスは適宜変更）
        self.model = YOLO('/workspace/puck_detection/results/test1_run5/weights/best.pt')
        self.model.to('cuda')
        # サブスクライバ：カメラ画像
        self.sub_image = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)

        # パブリッシャ：バウンディングボックス描画画像
        self.pub_image = self.create_publisher(Image, '/puck_detector/image', 10)

        # パブリッシャ：検出したbbox座標（x_min,y_min,x_max,y_max）をFloat32MultiArrayで
        self.pub_bbox = self.create_publisher(Float32MultiArray, '/puck_detector/bboxes_raw_position', 10)
        # [x1, y1, x2, y2,   x1, y1, x2, y2,   ...]の形式で連続で出ていく。

    def image_callback(self, msg: Image):
        # ROS Image -> OpenCV BGR画像
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 処理速度上昇のために画像をリサイズするとボックスの座標も変わるのでやめておく

        # YOLOv8推論（結果のbboxは xyxy フォーマット）
        results = self.model(frame,device='cuda')

        # bboxデータ用配列
        bboxes = []

        # 検出結果をOpenCVで描画
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # box.xyxy: tensor([[x1, y1, x2, y2]])
                xyxy = box.xyxy.cpu().numpy().astype(int)[0]
                x1, y1, x2, y2 = xyxy

                # バウンディングボックス描画（緑色、厚さ2）
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                #cx=(x1+x2)/2.0
                #cy=(y1+y2)/2.0
                # bbox配列に追加
                bboxes.extend([float(x1), float(y1),float(x2),float(y2)])

        # OpenCV画像をROS Imageに変換してpublish
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header  # 元画像のヘッダをコピー
        self.pub_image.publish(out_msg)

        # バウンディングボックス座標をFloat32MultiArrayでpublish
        bbox_msg = Float32MultiArray()
        bbox_msg.data = bboxes
        self.pub_bbox.publish(bbox_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PuckDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
