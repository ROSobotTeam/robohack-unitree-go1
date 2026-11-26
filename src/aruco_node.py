#!/usr/bin/env python3
import json
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()

        # Словарь ArUco по ТЗ: DICT_4X4_50 (совместимо с OpenCV 4.5 и 4.7+)
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Субпиксельное уточнение углов для надёжного поиска на кадрах 464x400
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.use_new_api = hasattr(cv2.aruco, 'ArucoDetector')
        if self.use_new_api:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Профиль с глубиной очереди 1: берём только самый свежий кадр, чтобы не грузить CPU
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.camera_topics = [
            '/camera/image_raw',
            '/camera_chin/image_raw',
            '/camera_left/image_raw',
            '/camera_right/image_raw',
            '/camera_rearDown/image_raw',
        ]
        self.detected_by_cam = {topic: set() for topic in self.camera_topics}

        for topic in self.camera_topics:
            self.create_subscription(
                Image,
                topic,
                lambda msg, t=topic: self.image_callback(msg, t),
                cam_qos
            )

        self.pub = self.create_publisher(String, '/aruco/detections', 10)
        self.timer = self.create_timer(0.1, self.publish_detections)  # 10 Гц
        self.get_logger().info('ArUco detector node started (5 cameras, DICT_4X4_50, depth=1)')

    def image_callback(self, msg: Image, topic: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            if self.use_new_api:
                _, ids, _ = self.detector.detectMarkers(gray)
            else:
                _, ids, _ = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )

            current_ids = set()
            if ids is not None:
                for marker_id in ids.flatten():
                    current_ids.add(int(marker_id))
            self.detected_by_cam[topic] = current_ids
        except Exception as e:
            self.get_logger().warn(f'Failed to process {topic}: {e}')

    def publish_detections(self):
        all_ids = sorted(set().union(*self.detected_by_cam.values()))
        out = String()
        out.data = json.dumps({'ids': all_ids})
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
