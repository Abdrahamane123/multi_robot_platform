import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class CameraNavigation(Node):
    def __init__(self):
        super().__init__('camera_navigation_node')

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        # Control parameters
        self.kp = 0.01  # Proportional gain for steering
        self.center_line = None  # Center line of the corridor
        self.image_width = 640  # Default image width (adjust based on your camera)
        self.image_height = 480  # Default image height (adjust based on your camera)

        self.get_logger().info("Camera Navigation Node Started")

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        # Process the image to detect the corridor
        processed_image, error = self.process_image(cv_image)

        # Publish velocity commands based on the error
        self.publish_velocity(error)

        # Display the processed image (for debugging)
        cv2.imshow("Processed Image", processed_image)
        cv2.waitKey(1)

    def process_image(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detect edges using Canny
        edges = cv2.Canny(blurred, 50, 150)

        # Detect lines using Hough Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=100)

        # Initialize variables for left and right lines
        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0

                # Classify lines as left or right based on slope
                if slope < -0.5:  # Left line
                    left_lines.append(line[0])
                elif slope > 0.5:  # Right line
                    right_lines.append(line[0])

                # Draw all detected lines (for debugging)
                cv2.line(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        # Calculate the average position of left and right lines
        left_avg = np.mean(left_lines, axis=0) if left_lines else None
        right_avg = np.mean(right_lines, axis=0) if right_lines else None

        # Draw the center line (red line)
        if left_avg is not None and right_avg is not None:
            x1_left, y1_left, x2_left, y2_left = left_avg
            x1_right, y1_right, x2_right, y2_right = right_avg

            # Calculate the midpoint between left and right lines
            x1_mid = int((x1_left + x1_right) // 2)
            x2_mid = int((x2_left + x2_right) // 2)
            y1_left = int(y1_left)
            y2_left = int(y2_left)

            # Draw the center line
            cv2.line(image, (x1_mid, y1_left), (x2_mid, y2_left), (0, 0, 255), 2)

            # Calculate the error (deviation from the center of the image)
            self.center_line = (x1_mid + x2_mid) // 2
            error = self.center_line - self.image_width // 2
        else:
            error = 0  # No lines detected, stop the robot

        return image, error

    def publish_velocity(self, error):
        cmd_vel = Twist()

        # Adjust angular velocity based on the error
        cmd_vel.angular.z = -self.kp * error

        # Move forward at a constant speed
        cmd_vel.linear.x = 0.2 if abs(error) < 50 else 0.1  # Slow down if error is large

        # Publish the velocity command
        self.cmd_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
