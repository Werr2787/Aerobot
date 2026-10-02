import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from uav_controller_ardupilot import UavControllerArduPilot

FLIGHT_ALTITUDE = 2.0
START_POINT = (0.0, 0.0, FLIGHT_ALTITUDE)
SEARCH_END_POINT = (5.0, 11.0, FLIGHT_ALTITUDE)
SEARCH_STEP_M = 0.75
HOLD_SECONDS = 10.0

# Same approximate intrinsics used by task2_object_search.py.
CAM_FX, CAM_FY = 554.3, 554.3
CAM_CX, CAM_CY = 320.0, 240.0
MIN_CIRCLE_AREA = 300
MIN_CIRCULARITY = 0.65


class DownwardCamera:
    def __init__(self, node):
        self.bridge = CvBridge()
        self.frame = None
        self.last_frame_time = 0.0
        node.create_subscription(Image, "/uav1/camera_down", self._callback, 10)

    def _callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.last_frame_time = time.monotonic()


def find_red_circle(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = cv2.inRange(hsv, np.array((0, 100, 100)), np.array((10, 255, 255)))
    upper_red = cv2.inRange(hsv, np.array((170, 100, 100)), np.array((180, 255, 255)))
    mask = cv2.bitwise_or(lower_red, upper_red)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    circles = []
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        if area < MIN_CIRCLE_AREA or perimeter == 0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < MIN_CIRCULARITY:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        circles.append((area, center))

    return max(circles, default=(0, None), key=lambda item: item[0])[1]


def estimate_circle_position(pixel, uav):
    px, py = pixel
    position = uav.pose.pose.position
    height = max(0.0, position.z)
    offset_x = (px - CAM_CX) * height / CAM_FX
    offset_y = (py - CAM_CY) * height / CAM_FY
    return position.x + offset_x, position.y + offset_y, FLIGHT_ALTITUDE


def wait_for_camera_frame(uav, camera, timeout=5.0):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and camera.frame is None and time.monotonic() < deadline:
        rclpy.spin_once(uav, timeout_sec=0.1)
    return camera.frame is not None


def search_route(uav, camera):
    start_x, start_y, _ = START_POINT
    end_x, end_y, _ = SEARCH_END_POINT
    distance = math.dist((start_x, start_y), (end_x, end_y))
    steps = max(1, math.ceil(distance / SEARCH_STEP_M))

    uav.get_logger().info("Ищу красный круг камерой, направленной вниз.")
    for step in range(1, steps + 1):
        fraction = step / steps
        waypoint = (
            start_x + (end_x - start_x) * fraction,
            start_y + (end_y - start_y) * fraction,
            FLIGHT_ALTITUDE,
        )
        if not uav.goto(*waypoint, hold_sec=0.2, tol=0.25, timeout=30.0):
            uav.get_logger().error("Не удалось продолжить поиск по маршруту.")
            return None

        if camera.frame is None or time.monotonic() - camera.last_frame_time > 1.0:
            continue
        pixel = find_red_circle(camera.frame)
        if pixel is not None:
            target = estimate_circle_position(pixel, uav)
            uav.get_logger().info(
                f"Найден красный круг; лечу над ним в точку "
                f"({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f})."
            )
            return target

    return None


def main():
    rclpy.init()
    uav = UavControllerArduPilot("uav1")
    camera = DownwardCamera(uav)

    try:
        if not uav.run_mission(altitude=FLIGHT_ALTITUDE):
            uav.get_logger().error("Взлёт не удался, завершаю миссию.")
            return

        if not wait_for_camera_frame(uav, camera):
            uav.get_logger().error("Нет изображения с камеры, поиск отменён.")
        else:
            target = search_route(uav, camera)
            if target is None:
                uav.get_logger().warn("Красный круг не найден на маршруте поиска.")
            elif not uav.goto(*target, hold_sec=HOLD_SECONDS, tol=0.25):
                uav.get_logger().error("Не удалось зависнуть над красным кругом.")

        if not uav.goto(*START_POINT, tol=0.3):
            uav.get_logger().error("Не удалось вернуться в стартовую точку.")
        uav.land()
    finally:
        uav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
