import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

import cv2
import numpy as np


class BoxTracker(Node):
    def __init__(self):
        super().__init__('box_tracker')

        # Publishes detected object position and category/color
        self.publisher_ = self.create_publisher(
            PointStamped,
            '/conveyor/object_position',
            10
        )

        # ==============================
        # Camera setup
        # ==============================
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('timer_period', 0.03)
        self.declare_parameter('object_height_cm', 6.0)

        camera_index = self.get_parameter('camera_index').value
        timer_period = self.get_parameter('timer_period').value

        self.cap = cv2.VideoCapture(camera_index)

        # Your calibration is for 640x480, so force the camera to this resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera index {camera_index}")
        else:
            self.get_logger().info(f"Camera opened successfully: index {camera_index}")

        # ==============================
        # Camera intrinsic calibration
        # ==============================
        self.K = np.array([
            [625.03791, 0.0, 332.75322],
            [0.0, 625.27446, 253.06032],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.D = np.array([
            -0.299634,
            -0.018828,
            -0.000560,
            -0.001331,
            0.000000
        ], dtype=np.float64)

        self.new_K = np.array([
            [562.034, 0.0, 334.85248],
            [0.0, 592.8009, 253.86692],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.map1 = None
        self.map2 = None

        self.get_logger().info("Camera intrinsic calibration loaded from code.")

        # ==============================
        # ArUco marker ID configuration
        # ==============================
        self.id_origin = 3       # Bottom-Right, origin (0,0)
        self.id_top_right = 2    # Upper-Right
        self.id_bottom_left = 1  # Bottom-Left
        self.id_top_left = 0     # Upper-Left

        self.marker_pixels = {
            self.id_origin: None,
            self.id_top_right: None,
            self.id_bottom_left: None,
            self.id_top_left: None
        }

        # Physical conveyor points in cm
        self.physical_pts = np.array([
            [0.0, 0.0],       # ID 3: Origin / Bottom-Right
            [0.0, 25.5],      # ID 2: Upper-Right
            [101.0, 0.0],     # ID 1: Bottom-Left
            [101.0, 25.5]     # ID 0: Upper-Left
        ], dtype="float32")

        self.transform_matrix = None

        # ==============================
        # ArUco detector setup
        # ==============================
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_5X5_50
            )
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.aruco_dict,
                self.aruco_params
            )
            self.use_new_aruco = True
            self.get_logger().info("Using new OpenCV ArUco detector.")
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(
                cv2.aruco.DICT_5X5_50
            )
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_aruco = False
            self.get_logger().info("Using old OpenCV ArUco detector.")

        # Timer instead of ROS Image subscription
        self.timer = self.create_timer(timer_period, self.process_frame)

    def undistort_frame(self, frame):
        h, w = frame.shape[:2]

        if self.map1 is None or self.map2 is None:
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.K,
                self.D,
                None,
                self.new_K,
                (w, h),
                cv2.CV_16SC2
            )

            self.get_logger().info("Undistortion maps created.")

        return cv2.remap(
            frame,
            self.map1,
            self.map2,
            interpolation=cv2.INTER_LINEAR
        )

    def process_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        # 1. Correct lens distortion first
        frame = self.undistort_frame(frame)

        # ==========================================
        # 1. ARUCO PERSPECTIVE CALIBRATION
        # ==========================================
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.use_new_aruco:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params
            )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                c = corners[i][0]

                if marker_id in self.marker_pixels:
                    cx = int(np.mean(c[:, 0]))
                    cy = int(np.mean(c[:, 1]))
                    self.marker_pixels[marker_id] = [cx, cy]

        # Build homography once all four required markers are detected
        if all(pt is not None for pt in self.marker_pixels.values()):
            src_pts = np.array([
                self.marker_pixels[self.id_origin],
                self.marker_pixels[self.id_top_right],
                self.marker_pixels[self.id_bottom_left],
                self.marker_pixels[self.id_top_left]
            ], dtype="float32")

            self.transform_matrix = cv2.getPerspectiveTransform(
                src_pts,
                self.physical_pts
            )

            poly_pts = np.array([
                self.marker_pixels[self.id_top_left],
                self.marker_pixels[self.id_top_right],
                self.marker_pixels[self.id_origin],
                self.marker_pixels[self.id_bottom_left]
            ], np.int32).reshape((-1, 1, 2))

            cv2.polylines(
                frame,
                [poly_pts],
                isClosed=True,
                color=(0, 0, 255),
                thickness=2
            )

            cv2.putText(
                frame,
                "ORIGIN ID 3 (0,0)",
                tuple(self.marker_pixels[self.id_origin]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2
            )

        # ==========================================
        # 2. COLOR TRACKING
        # ==========================================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color_targets = [
            ("Blue",  np.array([100, 180, 110]), np.array([125, 255, 255]), (255, 0, 0)),
            ("Green", np.array([40, 100, 100]),  np.array([85, 255, 255]),  (0, 255, 0)),
            ("Red",   np.array([0, 120, 70]),    np.array([10, 255, 255]),  (0, 0, 255)),
            ("Red",   np.array([170, 120, 70]),  np.array([180, 255, 255]), (0, 0, 255))
        ]

        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for name, lower, upper, draw_color in color_targets:
            mask = cv2.inRange(hsv, lower, upper)
            combined_mask = cv2.bitwise_or(combined_mask, mask)

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:
                area = cv2.contourArea(cnt)

                if 300 < area < 10000:
                    x, y, w, h = cv2.boundingRect(cnt)

                    if h == 0:
                        continue

                    aspect_ratio = float(w) / h

                    if 0.4 < aspect_ratio < 2.5:
                        cx = x + w // 2
                        cy = y + h // 2

                        if self.transform_matrix is not None:
                            pixel_pt = np.array([[[cx, cy]]], dtype="float32")

                            real_pt = cv2.perspectiveTransform(
                                pixel_pt,
                                self.transform_matrix
                            )

                            real_x = float(real_pt[0][0][0])
                            real_y = float(real_pt[0][0][1])

                            # Keep only detections inside conveyor area
                            if -2.0 <= real_x <= 103.0 and -2.0 <= real_y <= 27.5:
                                object_height_cm = float(
                                    self.get_parameter('object_height_cm').value
                                )

                                msg = PointStamped()
                                msg.header.stamp = self.get_clock().now().to_msg()

                                # Category/color is stored here
                                msg.header.frame_id = name

                                # Position in cm relative to conveyor origin
                                msg.point.x = real_x
                                msg.point.y = real_y

                                # Object height in cm
                                msg.point.z = object_height_cm

                                self.publisher_.publish(msg)

                                cv2.rectangle(
                                    frame,
                                    (x, y),
                                    (x + w, y + h),
                                    draw_color,
                                    2
                                )

                                label = f"{name} X:{real_x:.1f} Y:{real_y:.1f} Z:{object_height_cm:.1f}"

                                cv2.putText(
                                    frame,
                                    label,
                                    (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    draw_color,
                                    2
                                )

        cv2.imshow("Final Conveyor Tracking", frame)
        cv2.imshow("All Colors Mask", combined_mask)
        cv2.waitKey(1)

    def destroy_node(self):
        if hasattr(self, "cap"):
            self.cap.release()

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = BoxTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

