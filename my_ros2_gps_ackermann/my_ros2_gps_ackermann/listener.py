import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from gps_msgs.msg import GPSFix
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math

def calculate_bearing(lat1, lon1, lat2, lon2):
  """
  Calculates the bearing (angle) between two GPS coordinates.

  Args:
    lat1: Latitude of the starting point (degrees).
    lon1: Longitude of the starting point (degrees).
    lat2: Latitude of the ending point (degrees).
    lon2: Longitude of the ending point (degrees).

  Returns:
    The bearing (angle) in degrees.
  """

  # Convert latitude and longitude to radians
  lat1_rad = math.radians(lat1)
  lon1_rad = math.radians(lon1)
  lat2_rad = math.radians(lat2)
  lon2_rad = math.radians(lon2)

  # Calculate the difference in longitude
  dlon = lon2_rad - lon1_rad

  # Calculate the bearing
  bearing = math.atan2(math.sin(dlon) * math.cos(lat2_rad),
                       math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))

  # Convert to degrees
  bearing_degrees = math.degrees(bearing)

  return bearing_degrees

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great-circle distance between two points on the Earth.
    
    Parameters:
        lat1, lon1: Latitude and longitude of the first point in decimal degrees.
        lat2, lon2: Latitude and longitude of the second point in decimal degrees.

    Returns:
        Distance in meters.
    """
    R = 6371000  # Earth's radius in meters

    # Convert latitude and longitude from degrees to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    # Haversine formula
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # Distance in meters

class GPSAckermannNode(Node):
    def __init__(self):
        super().__init__('gps_ackermann_node')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.get_logger().info("updated")

        # Subscriber for real time gps data of robot
        self.create_subscription(
            GPSFix,
            '/gps_fix',
            self.update_robot_gps_pos,
            qos_profile
        )

        # Subscriber for real time gps data of robot
        self.create_subscription(
            GPSFix,
            '/gps/goal',
            self.recv_gps_goal,
            qos_profile
        )

        # Publisher for AckermannDriveStamped messages
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

        # Timer to publish Ackermann messages at 50 Hz (0.02s interval)
        self.timer = self.create_timer(0.02, self.publish_ackermann_message)
        self.goal_recv = False
        self.robot_lat = 0
        self.robot_lon = 0

    def update_robot_gps_pos(self, msg):
        """ Callback function for GPS messages """
        self.get_logger().info(f"current robot pos: latitude={msg.latitude}, longitude={msg.longitude}, altitude={msg.altitude}")
        self.robot_lat = msg.latitude
        self.robot_lon = msg.longitude
 
    def recv_gps_goal(self, msg):
        self.get_logger().info(f"received gps goal: latitude={msg.latitude}, longitude={msg.longitude}, altitude={msg.altitude}")
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude
        self.bearing = calculate_bearing(self.robot_lat, self.robot_lon, msg.latitude, msg.longitude)
        self.get_logger().info(f"bearing={self.bearing}")
        if self.bearing < 0:
            self.get_logger().info("Bearing angle is negative, ignoring goal. Currently only goals that are to the right of robot will work")
            return
        if self.bearing > 23.0:
           self.bearing = 23.0
        self.goal_recv = True


    def publish_ackermann_message(self):
        """ Publishes AckermannDriveStamped messages at 50 Hz """
        if self.goal_recv != True:
            return

        msg = AckermannDriveStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
        msg.header.frame_id = "base_link"  # Set reference frame

        # Set constant drive values
        msg.drive.speed = 0.5
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        """ expressed as a percentage of the max steering angle"""
        msg.drive.steering_angle = self.bearing/23.0
        msg.drive.steering_angle_velocity = 0.0

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: Speed={msg.drive.speed}, Steering Angle={msg.drive.steering_angle}")

        """ Check goal distance"""
        self.goal_error = haversine_distance(self.goal_lat, self.goal_lon, self.robot_lat, self.robot_lon)
        self.get_logger().info(f"Goal error: {self.goal_error}")
        if self.goal_error < 3.0:
            self.get_logger().info("Goal acheived within bound of 3m")
            self.goal_recv = False
            # Zero out
            msg.drive.speed = 0.0
            msg.drive.acceleration = 0.0
            msg.drive.jerk = 0.0
            """ expressed as a percentage of the max steering angle"""
            msg.drive.steering_angle = 0.0
            msg.drive.steering_angle_velocity = 0.0

def main():
    rclpy.init()
    node = GPSAckermannNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
