#!/usr/bin/env python
# ROS 2 imports
import rclpy  # Core ROS 2 Python library
from rclpy.node import Node  # Node class for creating ROS 2 nodes

# Message types we'll publish
from sensor_msgs.msg import Image  # Standard ROS Image message
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose  # For detection messages
from std_msgs.msg import Header  # Standard message header for timestamp/frame info

# OpenCV bridge for converting images between ROS and OpenCV
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

# Roboflow OAK-D Lite interface
from roboflowoak import RoboflowOak


class HumanDetectionNode(Node):
    def __init__(self):
        # Initialize the ROS 2 node with the name 'human_detection_node'
        super().__init__('human_detection_node')

        # === ROS 2 PUBLISHERS ===
        # Publisher for the 2D detection array message
        self.detection_pub = self.create_publisher(
            Detection2DArray,         # Message type
            '/human_detections',      # Topic name
            10                        # Queue size
        )

        # Publisher for the annotated image (detections drawn on it)
        self.image_pub = self.create_publisher(
            Image,
            '/human_detections/image',
            10
        )

        # Publisher for the depth map (grayscale for visualization)
        self.depth_pub = self.create_publisher(
            Image,
            '/human_detections/depth',
            10
        )

        # === CV BRIDGE ===
        # Used to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # === ROBOFLOWOAK INITIALIZATION ===
        # This creates an instance of RoboflowOak and connects to your model
        self.rf = RoboflowOak(
            #model="person-snpao",
                                            # Model name from Roboflow
            model="person-lying-down",
            confidence=0.05,                # Confidence threshold
            overlap=0.5,                    # NMS overlap threshold
            #version="2",                   
                                            # Model version
            version="2",   
            api_key="R4jbOhEOxwSSDOBryrhH", # Your Roboflow API key
            rgb=True,                       # Whether to use RGB stream
            depth=True,                     # Whether to use depth stream
            device=None,                    # Auto device selection
            blocking=True                   # Blocking detection call
        )

        # === CREATE A TIMER CALLBACK ===
        # Calls self.run_model every 0.1 seconds (10Hz)
        self.create_timer(0.1, self.run_model)

    def run_model(self):
        """
        This function runs periodically at the timer rate.
        It runs inference on the OAK-D Lite, processes predictions,
        and publishes them as ROS 2 messages.
        """
        t0 = time.time()  # For FPS calculation

        # === RUN INFERENCE ===
        # result: dictionary of prediction data
        # frame: image with detections drawn on it
        # raw_frame: raw image without annotations
        # depth: depth map aligned with raw_frame
        result, frame, raw_frame, depth = self.rf.detect()

        # Extract predictions from result dictionary
        predictions = result["predictions"]

        # === PUBLISH DETECTION MESSAGES ===
        detections_msg = self.create_detection_msg(predictions, raw_frame)
        self.detection_pub.publish(detections_msg)

        # === PUBLISH IMAGE FRAME (with bounding boxes drawn on it) ===
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(image_msg)

        # === PUBLISH DEPTH MAP (normalized for visualization purposes) ===
        if depth is not None:
            max_depth = np.amax(depth)  # Get the maximum depth for normalization
            # Normalize depth map to 0-255 (uint8), so it can be visualized
            depth_norm = (depth / max_depth * 255).astype(np.uint8)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_norm, encoding='mono8')
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            self.depth_pub.publish(depth_msg)

        # === PRINT INFO TO CONSOLE ===
        t = time.time() - t0  # How long it took to run one detection cycle
        self.get_logger().info(f'FPS: {1/t:.2f}, Published {len(predictions)} detections')

    def create_detection_msg(self, predictions, frame):
        """
        Converts the list of predictions from Roboflow into a ROS 2 Detection2DArray message.
        """
        # Create the detection array message
        detection_array = Detection2DArray()

        # Populate standard header (timestamp + frame ID)
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'oakd_camera_frame'  # This should match your TF tree if you have one

        # Iterate over each prediction and convert it to Detection2D
        for pred in predictions:
            det = Detection2D()

            # Add the header (time + frame ID)
            det.header = detection_array.header

            # Bounding box center and size (center_x, center_y, size_x, size_y)
            center_x = pred.x
            center_y = pred.y
            width = pred.width
            height = pred.height

            det.bbox.center.position.x = center_x
            det.bbox.center.position.y = center_y
            det.bbox.size_x = width
            det.bbox.size_y = height

            # Check if the class ID is "0" (human class)
            if pred.class_id == 0:  # If class is 0, it means "human"
                # Create a hypothesis to store class + confidence score
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = "human"  # Class "human"
                hypothesis.hypothesis.score = float(pred.confidence)  # Confidence score

                # Append hypothesis to results (allows multiple hypotheses, e.g. multi-label)
                det.results.append(hypothesis)

            # Add the detection to the array
            detection_array.detections.append(det)

        return detection_array


def main(args=None):
    """
    Main entry point for the ROS 2 node.
    """
    # Initialize ROS 2 Python
    rclpy.init(args=args)

    # Create an instance of the node
    node = HumanDetectionNode()

    # Keep the node alive and spinning (listening and publishing)
    rclpy.spin(node)

    # Cleanup when shutting down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # If the script is run directly (not imported), call main()
    main()
