#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
from datetime import datetime
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import NavSatFix
from rclpy.time import Time

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')

        # Initialize last GPS location (set to None initially)
        self.last_gps = None
        # New: Timestamp of last log
        self.last_gps_log_time = self.get_clock().now()

        # Create a dedicated log directory
        log_dir = os.path.join(os.getcwd(), 'logs')
        os.makedirs(log_dir, exist_ok=True)  # Create the directory if it doesn't exist

        # Path to the log file
        self.file_path = os.path.join(log_dir, 'detections_log.csv')

        # Subscribe to the detection topic
        self.create_subscription(
            Detection2DArray,
            '/human_detections',   # <-- match this with the publisher topic
            self.detection_callback,
            10                     # QoS queue size
        )

        # Subscribe to the GPS fix topic
        self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        self.get_logger().info("Recorder node started! Listening to /human_detections and /gps/fix...")

    def detection_callback(self, msg):
        """Callback for handling Detection2DArray messages."""
        detections = msg.detections

        if not detections:
            self.get_logger().info("No detections in this message.")
            return

        for idx, detection in enumerate(detections):
            for result in detection.results:
                class_id = result.hypothesis.class_id
                confidence = result.hypothesis.score

                # Normalize class_id into a lowercase string for comparison
                class_label_str = str(class_id).lower()

                # Identify class label
                if class_label_str == 'human' or class_id == 0:
                    label = 'human'
                elif class_label_str == 'vehicle' or class_id == 1:
                    label = 'vehicle'
                else:
                    label = f"unknown({class_id})"

                if self.last_gps:
                    latitude = self.last_gps.latitude
                    longitude = self.last_gps.longitude
                    altitude = self.last_gps.altitude

                    self.get_logger().info(
                        f"[Detection {idx}] Class: {label}, Confidence: {confidence:.2f}, "
                        f"Location: Latitude: {latitude:.6f}, Longitude: {longitude:.6f}, Altitude: {altitude:.2f}m"
                    )

                    # Log detection to file
                    self.log_to_file(class_id, label, confidence, latitude, longitude, altitude)

                else:
                    self.get_logger().warn(
                        f"[Detection {idx}] Class: {label}, Confidence: {confidence:.2f}, No GPS data available."
                    )

    def gps_callback(self, msg):
        """Callback for handling NavSatFix (GPS) messages."""
        self.last_gps = msg
        current_time = self.get_clock().now()

        # Duration in seconds between logs
        log_interval_sec = 5.0

        if (current_time - self.last_gps_log_time).nanoseconds * 1e-9 >= log_interval_sec:
            self.get_logger().info(
                f"GPS Update: Lat: {msg.latitude:.6f}, Lon: {msg.longitude:.6f}, Alt: {msg.altitude:.2f}m"
            )
            self.last_gps_log_time = current_time

    def log_to_file(self, class_id, class_label, confidence, latitude, longitude, altitude):
        """Logs detection to CSV file."""
        file_exists = os.path.isfile(self.file_path)

        current_time = datetime.utcnow().isoformat()

        try:
            with open(self.file_path, 'a') as file:
                if not file_exists:
                    # Write the header if it's a new file
                    file.write('timestamp,class_id,class_label,confidence,latitude,longitude,altitude\n')

                file.write(f"{current_time},{class_id},{class_label},{confidence:.2f},{latitude:.6f},{longitude:.6f},{altitude:.2f}\n")

            self.get_logger().info(f"Logged detection to CSV: {class_label}")

        except Exception as e:
            self.get_logger().error(f"Error writing to file: {e}")

def main(args=None):
    rclpy.init(args=args)

    recorder = RecorderNode()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Recorder node stopped by user.")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
