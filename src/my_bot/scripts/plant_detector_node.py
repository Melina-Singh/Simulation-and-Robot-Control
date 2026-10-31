#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import time

class PlantDetector(Node):
    def __init__(self):
        super().__init__('plant_detector')
        self.bridge = CvBridge()

        pkg_path = get_package_share_directory('my_bot')
        model_dir = os.path.join(pkg_path, 'models')
        proto_path = os.path.join(model_dir, 'deploy.prototxt')
        model_path = os.path.join(model_dir, 'mobilenet_iter_73000.caffemodel')
        self.net = cv2.dnn.readNetFromCaffe(proto_path, model_path)

        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant",
                        "sheep", "sofa", "train", "tvmonitor"]
        self.target_class = "pottedplant"
        self.target_class_id = self.CLASSES.index(self.target_class)

        self.subscription = self.create_subscription(Image, '/plant_cam/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_image_pub = self.create_publisher(Image, '/plant_detection/image', 10)
        self.spray_pub = self.create_publisher(Empty, '/spray_cmd', 10)
        self.last_spray_time = time.time()

        self.get_logger().info("🍅 Tomato Plant Detector (max distance) Initialized")

    def has_green_vertical_stem(self, frame, tomato_box):
        x, y, w, h = tomato_box
        roi = frame[y:y+h, x:x+w]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_roi, (35, 40, 40), (85, 255, 255))
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            xg, yg, wg, hg = cv2.boundingRect(cnt)
            aspect_ratio = hg / (wg + 1e-5)
            if aspect_ratio > 1.2 and hg > 6:  # Less strict for small/distant
                return True
        return False

    def detect_pottedplants_ssd(self, frame):
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()
        boxes = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            idx = int(detections[0, 0, i, 1])
            if idx == self.target_class_id and confidence > 0.45:  # Lowered for sensitivity
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                boxes.append((startX, startY, endX-startX, endY-startY, confidence))
        return boxes

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red1 = cv2.inRange(hsv, (0, 80, 40), (10, 255, 255))
        red2 = cv2.inRange(hsv, (160, 80, 40), (180, 255, 255))
        red_mask = cv2.bitwise_or(red1, red2)

        tomato_candidates = []
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 40:  # Lower threshold: catch small/far blobs!
                continue
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            if self.has_green_vertical_stem(frame, (x, y, w_box, h_box)):
                tomato_candidates.append((x, y, w_box, h_box, area))

        # Add SSD-based candidates
        for box in self.detect_pottedplants_ssd(frame):
            x, y, w_box, h_box, conf = box
            tomato_candidates.append((x, y, w_box, h_box, 9999))  # Large area, will prioritize visual

        if tomato_candidates:
            # Select the *highest* (farthest, i.e. smallest y)
            farthest = min(tomato_candidates, key=lambda b: b[1])
            x, y, w_box, h_box, _ = farthest

            cv2.rectangle(frame, (x, y), (x + w_box, y + h_box), (0, 0, 255), 2)
            cv2.putText(frame, "Tomato Plant", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            self.get_logger().info(f"🍅 Tomato plant detected at ({x + w_box//2}, {y + h_box//2})")

            # # Movement logic: turn toward center, drive forward if not centered
            # center_x = x + w_box // 2
            # image_center = frame.shape[1] // 2
            # twist = Twist()
            # if abs(center_x - image_center) > 40:
            #     twist.angular.z = -0.2 if center_x < image_center else 0.2
            # else:
            #     twist.linear.x = 0.1
            # self.publisher.publish(twist)

            # Spray if close enough (based on box size or y+h_box proximity)
            if h_box > h//4 and time.time() - self.last_spray_time > 3.0:
                self.spray_pub.publish(Empty())
                self.get_logger().info("💦 Spray command sent!")
                self.last_spray_time = time.time()
        else:
            stop = Twist()
            self.publisher.publish(stop)

        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.debug_image_pub.publish(debug_msg)

        try:
            cv2.imshow("Camera View", frame)
            cv2.waitKey(1)
        except cv2.error:
            self.get_logger().warn("cv2.imshow failed — try using rqt_image_view")

def main(args=None):
    rclpy.init(args=args)
    node = PlantDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
