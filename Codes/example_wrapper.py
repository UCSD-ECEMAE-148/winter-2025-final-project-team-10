import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from roboflowoak import RoboflowOak

class YourNode(Node):
    def __init__(self):
        super().__init__('your_node')

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/your_detections', 10)
        self.image_pub = self.create_publisher(Image, '/yoour_detections/image', 10)
        self.depth_pub = self.create_publisher(Image, '/your_detections/depth', 10)

        # CV bridge
        self.bridge = CvBridge()

        # Initialize RoboflowOak
        self.rf = RoboflowOak(
            model="your_model_name",
            confidence=0.05, # value up to you
            overlap=0.5,     # value up to you
            version="1",     # ensure it's the right value
            api_key="your_api",
            rgb=True,
            depth=True,
            device=None,
            blocking=True
        )

    def run_model(self):
        t0 = time.time()

        # Run inference
        result, frame, raw_frame, depth = self.rf.detect()

        predictions = result["predictions"]

        # Publish detections
        detections_msg = self.create_detection_msg(predictions, raw_frame)
        self.detection_pub.publish(detections_msg)

        # Publish images
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(image_msg)

        # Publish depth map
        if depth is not None:
            max_depth = np.amax(depth)
            depth_norm = (depth / max_depth * 255).astype(np.uint8)  # Normalize for visualization
            depth_msg = self.bridge.cv2_to_imgmsg(depth_norm, encoding='mono8')
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            self.depth_pub.publish(depth_msg)

        t = time.time() - t0
        self.get_logger().info(f'FPS: {1/t:.2f}, Published {len(predictions)} detections')

    def create_detection_msg(self, predictions, frame):
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'oakd_camera_frame'  # or whatever frame you want

        for pred in predictions:
            det = Detection2D()

            # Set header
            det.header = detection_array.header

            # Bounding box (center_x, center_y, size_x, size_y)
            center_x = pred.x
            center_y = pred.y
            width = pred.width
            height = pred.height

            det.bbox.center.position.x = center_x
            det.bbox.center.position.y = center_y
            det.bbox.size_x = width
            det.bbox.size_y = height

            # Hypothesis: confidence + class ID (optional pose)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = pred.class_name
            hypothesis.hypothesis.score = float(pred.confidence)

            det.results.append(hypothesis)

            detection_array.detections.append(det)

        return detection_array

def main(args=None):
    rclpy.init(args=args)
    node = YourNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

