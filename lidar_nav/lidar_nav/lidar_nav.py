import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from collections import deque

class LineDetector(Node):
    def __init__(self):
        super().__init__('parallel_nav_node')

        # Subscriptions and publishers
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.cmd_publisher = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.image_pub = self.create_publisher(Image, 'image_topic', 10)
        self.bridge = CvBridge()

        # Parameters
        self.frame_width = 300
        self.frame_height = 600
        # Convertisseur mètre -> pixel : on cartographie ±2m => total 4m -> frame_width pixels
        # Configure mapping area: forward distance and lateral span
        self.mapping_forward_m = 6.0  # meters ahead to map
        self.lane_width_m = 4.0        # expected distance between rows (meters)
        # pixel scale (separate for x (forward) and y (lateral))
        self.pixels_per_meter_x = self.frame_height / self.mapping_forward_m
        self.pixels_per_meter_y = self.frame_width / (2.0 * self.lane_width_m)
        self.lane_width_pixel = int(self.lane_width_m * self.pixels_per_meter_y)
        self.max_lidar_range = 16.0
        self.cmd_vel = Twist()

        # Moving average filter
        self.moving_avg_window = 3
        # Initialiser la fenêtre avec la valeur centre pour éviter NaN lors du calcul de la moyenne
        self.x_mid_history = deque([self.frame_width // 2] * self.moving_avg_window, maxlen=self.moving_avg_window)

        # Control parameters
        self.kp = 0.005  # Proportional gain
        self.kd = 0.001  # Derivative gain
        self.last_error = 0.0

        self.timer = self.create_timer(0.05, self.run)

        # --- Headland (zone de fourrière) U-turn state machine ---
        # States: 'NORMAL', 'FIRST_TURNING', 'CROSSING', 'SECOND_TURNING'
        self.headland_state = 'NORMAL'
        # +1 means we turned left first, -1 would mean right first (we enforce left first here)
        self.headland_dir = None
        # Simple debouncing counters to avoid false triggers
        self.headland_detect_count = 0
        self.headland_detect_threshold = 5
        # track if we recently had rows so we detect the transition into a headland
        self.headland_prev_had_rows = False
        # timing for smooth arc turns
        self.turn_start_time = None
        self.turn_duration = 1.2  # seconds, tuned for smooth arc
        # crossing timeout (allow existing navigation to traverse the inter-row)
        self.crossing_start_time = None
        self.crossing_timeout = 2.5  # seconds

        self.get_logger().info("Parallel Line Navigation Node Started")

    def scan_callback(self, msg):
        # Convert laser scan data to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        # Clean up invalid ranges: inf, nan or 0 -> set to max range
        invalid_mask = np.isinf(ranges) | np.isnan(ranges) | (ranges <= 0.0)
        ranges[invalid_mask] = self.max_lidar_range

        X = ranges * np.cos(angles)
        Y = ranges * np.sin(angles)

        # Segment zones
        self.process_zones(angles, ranges)

        self.process_lidar_data(X, Y)

    def process_zones(self, angles, ranges):
        # Define zone limits (ROS lidar: 0 = front, positive = left)
        front_min, front_max = -np.pi/6, np.pi/6
        left_min, left_max = np.pi/4, 3 * np.pi / 4
        right_min, right_max = -3 * np.pi / 4, -np.pi / 4

        self.scan_front = ranges[(angles >= front_min) & (angles <= front_max)]
        self.scan_left = ranges[(angles >= left_min) & (angles <= left_max)]
        self.scan_right = ranges[(angles >= right_min) & (angles <= right_max)]
        # Back zone wraps around ±π
        self.scan_back = ranges[(angles >= 3 * np.pi / 4) | (angles <= -3 * np.pi / 4)]

    def process_lidar_data(self, X, Y):
        self.cmd_vel = Twist()
        # Create blank image
        amap = np.ones((self.frame_height, self.frame_width, 3), np.uint8) * 255

        # Map Cartesian points to pixels
        # projection: x is forward (0..mapping_forward_m), y is lateral (-lane_width_m..+lane_width_m)
        y_min = -self.lane_width_m
        y_max = self.lane_width_m
        for x, y in zip(X, Y):
            if np.isnan(x) or np.isnan(y):
                continue
            # only keep points in the mapped window
            if x < 0.0 or x > self.mapping_forward_m:
                continue
            if y < y_min or y > y_max:
                continue
            px = int((y - y_min) * self.pixels_per_meter_y)
            py = int((self.mapping_forward_m - x) * self.pixels_per_meter_x)
            if 0 <= px < self.frame_width and 0 <= py < self.frame_height:
                amap[py, px] = (0, 0, 0)

        # Image processing: build binary mask from projected points and thicken blobs
        gray_img = cv2.cvtColor(amap, cv2.COLOR_BGR2GRAY)
        # invert threshold to get black points as white on mask
        _, mask = cv2.threshold(gray_img, 250, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Try to fit two parallel lines from the mask (more robust for sparse LiDAR)
        left_line = None
        right_line = None
        fitted = False
        fitted_lines = self.fit_parallel_lines_from_mask(mask)
        if fitted_lines is not None:
            left_line, right_line = fitted_lines
            fitted = True

        # If fitting failed, fallback to Hough lines
        lines = None
        if not fitted:
            edges = cv2.Canny(gray_img, 50, 200)
            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=25, minLineLength=10, maxLineGap=150)

            left_lines = []
            right_lines = []
            x_mid = self.frame_width // 2

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    x_avg = (x1 + x2) // 2
                    if x_avg < x_mid:
                        left_lines.append(line[0])
                    else:
                        right_lines.append(line[0])

                left_line = self.select_longest_line(left_lines)
                right_line = self.select_longest_line(right_lines)

        # Draw left and right lines if found
        if left_line is not None:
            cv2.line(amap, (int(left_line[0]), int(left_line[1])), (int(left_line[2]), int(left_line[3])), (255, 0, 0), 2)
        if right_line is not None:
            cv2.line(amap, (int(right_line[0]), int(right_line[1])), (int(right_line[2]), int(right_line[3])), (0, 255, 0), 2)

        # Calculate midline (Xmin(MID))
        x_mid = self.frame_width // 2
        x_min_mid = self.calculate_midline(left_line, right_line, x_mid)

        # Apply moving average filter to smooth Xmin(MID)
        self.x_mid_history.append(x_min_mid)
        x_min_mid_smoothed = np.mean(self.x_mid_history)

        # Draw midline (red line)
        mid_line_y1 = 0
        mid_line_y2 = self.frame_height
        mid_line_x1 = int(x_min_mid_smoothed)
        mid_line_x2 = int(x_min_mid_smoothed)
        cv2.line(amap, (mid_line_x1, mid_line_y1), (mid_line_x2, mid_line_y2), (0, 0, 255), 2)

        # Calculate error and adjust robot trajectory
        error = x_min_mid_smoothed - x_mid
        derivative_error = error - self.last_error
        self.last_error = error

        # Avoid empty arrays when checking obstacles
        front_has_obstacle = (hasattr(self, 'scan_front') and self.scan_front.size > 0 and np.any(self.scan_front < 0.4))
        if front_has_obstacle:  # Obstacle in front
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
        else:
            # Reduce angular gain if behaviour too oscillatory
            ang = self.kp * error + self.kd * derivative_error
            # cap angular speed
            ang = max(-1.0, min(1.0, ang))
            if abs(error) < 10:  # No significant deviation
                self.cmd_vel.linear.x = 0.25
                self.cmd_vel.angular.z = 0.0
            elif abs(error) < 50:  # Moderate deviation
                self.cmd_vel.linear.x = 0.2
                self.cmd_vel.angular.z = ang
            elif abs(error) < 100:  # Significant deviation
                self.cmd_vel.linear.x = 0.15
                self.cmd_vel.angular.z = ang
            else:  # Extreme deviation
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = ang

        # Publish velocity and image
        self.cmd_publisher.publish(self.cmd_vel)
        try:
            image_message = self.bridge.cv2_to_imgmsg(amap, encoding="bgr8")
            self.image_pub.publish(image_message)
        except Exception as e:
            self.get_logger().error(str(e))

    def select_longest_line(self, lines):
        if not lines:
            return None
        longest_line = max(lines, key=lambda l: np.hypot(l[2] - l[0], l[3] - l[1]))
        return longest_line

    # --- Headland detection and maneuver helpers ---
    def detect_headland(self, left_line, right_line):
        """Detect entering a headland when rows are lost after being previously seen.
        Debounced to avoid flicker. Requires some open space ahead (median front range).
        """
        has_rows = (left_line is not None) or (right_line is not None)
        # If rows are present, remember that and reset counters
        if has_rows:
            self.headland_prev_had_rows = True
            self.headland_detect_count = 0
            return False

        # No rows now. If we previously had rows, start counting consecutive frames with no rows
        if self.headland_prev_had_rows:
            self.headland_detect_count += 1
            if self.headland_detect_count >= self.headland_detect_threshold:
                # require reasonably open space ahead to be considered a headland
                front_ok = False
                if hasattr(self, 'scan_front') and self.scan_front.size > 0:
                    try:
                        front_ok = float(np.median(self.scan_front)) > 0.8
                    except Exception:
                        front_ok = False
                if front_ok:
                    # transition detected
                    self.headland_prev_had_rows = False
                    self.headland_detect_count = 0
                    return True
        return False

    def start_turn(self, first=True):
        """Initiate a smooth arc turn. If first=True we enforce a left turn, otherwise we take opposite direction.
        Stores headland_dir as +1 for left, -1 for right.
        """
        if first:
            self.headland_dir = +1
            self.headland_state = 'FIRST_TURNING'
        else:
            # opposite direction for second headland
            self.headland_dir = -1 if self.headland_dir is None else -self.headland_dir
            self.headland_state = 'SECOND_TURNING'
        self.turn_start_time = time.time()
        self.get_logger().info(f"Headland: starting {'left' if self.headland_dir>0 else 'right'} turn (state={self.headland_state})")

    def update_turn(self):
        """Compute and publish twist commands during a turning phase.
        Uses a smooth arc: small forward velocity and angular velocity scaled by remaining time.
        Returns True if turn is still in progress, False if completed.
        """
        if self.turn_start_time is None:
            return False
        elapsed = time.time() - self.turn_start_time
        if elapsed >= self.turn_duration:
            # finish turn – stop rotation but keep a small forward push for stability
            self.cmd_vel.linear.x = 0.05
            self.cmd_vel.angular.z = 0.0
            # finalize state transitions
            if self.headland_state == 'FIRST_TURNING':
                self.headland_state = 'CROSSING'
                self.crossing_start_time = time.time()
                self.get_logger().info('Headland: completed first turn, entering CROSSING')
            elif self.headland_state == 'SECOND_TURNING':
                self.headland_state = 'NORMAL'
                self.headland_dir = None
                self.crossing_start_time = None
                self.get_logger().info('Headland: completed second turn, returning to NORMAL navigation')
            self.turn_start_time = None
            return False

        # while turning: create smooth angular profile (ease-in/out)
        t = elapsed / self.turn_duration
        # ease-in-out factor
        ang_scale = 0.5 - 0.5 * np.cos(np.pi * t)
        max_ang = 0.9
        ang = self.headland_dir * max_ang * ang_scale
        # small forward velocity to create an arc (smoother than pure rotation)
        lin = 0.08 + 0.07 * (1 - abs(t - 0.5) * 2)  # slightly higher mid-turn
        self.cmd_vel.linear.x = lin
        self.cmd_vel.angular.z = ang
        # publish immediate command to ensure robot follows the maneuver
        self.cmd_publisher.publish(self.cmd_vel)
        return True

    def calculate_midline(self, left_line, right_line, x_mid):
        if left_line is not None and right_line is not None:
            left_x = (left_line[0] + left_line[2]) // 2
            right_x = (right_line[0] + right_line[2]) // 2
            return (left_x + right_x) // 2
        elif left_line is not None:
            left_x = (left_line[0] + left_line[2]) // 2
            return left_x + self.lane_width_pixel // 2
        elif right_line is not None:
            right_x = (right_line[0] + right_line[2]) // 2
            return right_x - self.lane_width_pixel // 2
        else:
            return x_mid

    def run(self):
        pass  # Placeholder for future features

    def detect_rows_from_points(self, X, Y):
        """Fallback: detect two row centers from raw LiDAR points using a lateral histogram.
        Returns the lateral midpoint (y in meters) between the two strongest peaks or None.
        """
        # Use points in a forward band
        mask = (X > 0.2) & (X < self.mapping_forward_m)
        ys = Y[mask]
        if ys.size < 30:
            return None

        # Histogram over lateral range
        bins = 120
        hist_range = (-self.lane_width_m, self.lane_width_m)
        hist, edges = np.histogram(ys, bins=bins, range=hist_range)
        if np.all(hist == 0):
            return None

        # find two largest peaks separated by at least 0.5 m
        peak_idxs = np.argsort(hist)[-2:]
        peak_centers = []
        bin_width = edges[1] - edges[0]
        for idx in peak_idxs:
            center = (edges[idx] + edges[idx + 1]) / 2.0
            peak_centers.append(center)
        if len(peak_centers) < 2:
            return None
        # ensure they are distinct
        if abs(peak_centers[0] - peak_centers[1]) < 0.5:
            return None

        # compute mean y of points near each peak to refine
        c1, c2 = peak_centers
        w = bin_width * 1.5
        cluster1 = ys[(ys > c1 - w) & (ys < c1 + w)]
        cluster2 = ys[(ys > c2 - w) & (ys < c2 + w)]
        if cluster1.size == 0 or cluster2.size == 0:
            return None
        yc1 = np.mean(cluster1)
        yc2 = np.mean(cluster2)
        return float((yc1 + yc2) / 2.0)

    def fit_parallel_lines_from_mask(self, mask):
        """Fit two approximately parallel lines from binary mask.
        Returns (left_line, right_line) each as [x1,y1,x2,y2] in image coords or None.
        """
        ys, xs = np.where(mask > 0)
        if xs.size < 30:
            return None

        x_mid = self.frame_width // 2
        left_idx = xs < x_mid
        right_idx = xs >= x_mid
        if np.count_nonzero(left_idx) < 10 or np.count_nonzero(right_idx) < 10:
            return None

        # Fit x = m*y + c for each side (handles near-vertical lines)
        try:
            m1, c1 = np.polyfit(ys[left_idx], xs[left_idx], 1)
            m2, c2 = np.polyfit(ys[right_idx], xs[right_idx], 1)
        except Exception:
            return None

        # enforce parallelism by averaging slopes
        m = (m1 + m2) / 2.0

        # recompute intercepts so each line passes through its side centroid
        y1_mean = float(np.mean(ys[left_idx]))
        x1_mean = float(np.mean(xs[left_idx]))
        c1 = x1_mean - m * y1_mean

        y2_mean = float(np.mean(ys[right_idx]))
        x2_mean = float(np.mean(xs[right_idx]))
        c2 = x2_mean - m * y2_mean

        # build line pixel coordinates across full image height
        y0 = 0
        y1 = self.frame_height - 1
        lx0 = int(max(0, min(self.frame_width - 1, m * y0 + c1)))
        lx1 = int(max(0, min(self.frame_width - 1, m * y1 + c1)))
        rx0 = int(max(0, min(self.frame_width - 1, m * y0 + c2)))
        rx1 = int(max(0, min(self.frame_width - 1, m * y1 + c2)))

        left_line = [lx0, y0, lx1, y1]
        right_line = [rx0, y0, rx1, y1]
        return left_line, right_line

def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()