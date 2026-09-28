"""
Задание 2: Поиск объектов.
Облетаем помещения по заданным точкам, на каждой ищем цветные объекты через
камеру вниз, переводим их пиксельные координаты в мировые (относительно
старта) с помощью камеры глубины, возвращаемся на старт.

Перед использованием:
  - заполните WAYPOINTS координатами обхода вашей сцены,
  - подставьте реальные внутренние параметры камеры (CAM_FX, CAM_FY, CAM_CX, CAM_CY)
    из топика /uav1/camera_down_info (или /uav1/camera_info, если ищете камерой вперёд),
  - при необходимости скорректируйте OFFSET_CAMERA — смещение камеры относительно
    центра дрона (обычно указано в описании модели дрона в сцене).
"""

import time
import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from uav_controller_ardupilot import UavControllerArduPilot

FLIGHT_ALTITUDE = 1.8

# TODO: заполните точками обхода вашей сцены (X, Y, Z)
WAYPOINTS = [
    (2.0, 0.0, FLIGHT_ALTITUDE),
    (4.0, 0.0, FLIGHT_ALTITUDE),
    (4.0, 3.0, FLIGHT_ALTITUDE),
    (2.0, 3.0, FLIGHT_ALTITUDE),
]

# TODO: подставьте реальные параметры из /uav1/camera_down_info
CAM_FX, CAM_FY = 554.3, 554.3
CAM_CX, CAM_CY = 320.0, 240.0

COLOR_RANGES = {
    "red":    [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))],
    "green":  [((40, 70, 70), (80, 255, 255))],
    "blue":   [((100, 100, 100), (130, 255, 255))],
    "yellow": [((20, 100, 100), (35, 255, 255))],
}


def detect_objects(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    found = []
    for color, ranges in COLOR_RANGES.items():
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < 300:
                continue
            approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
            shape = {3: "triangle", 4: "square", 5: "pentagon"}.get(len(approx), "circle")
            (cx, cy), _ = cv2.minEnclosingCircle(c)
            found.append({"color": color, "shape": shape, "pixel": (cx, cy)})
    return found


class CameraListener:
    """Держит последний BGR-кадр и последнюю глубинную карту (если нужна)."""

    def __init__(self, node, rgb_topic, depth_topic=None):
        self.bridge = CvBridge()
        self.frame = None
        self.depth = None
        node.create_subscription(Image, rgb_topic, self._rgb_cb, 10)
        if depth_topic:
            node.create_subscription(Image, depth_topic, self._depth_cb, 10)

    def _rgb_cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


def pixel_to_world(px, py, depth, drone_xyz):
    """
    Упрощённый пересчёт для камеры, смотрящей строго вниз:
    депроецируем пиксель в систему координат камеры, затем добавляем
    положение дрона (без учёта крена/тангажа — для точной работы на
    практике учтите ориентацию дрона через quaternion из uav.pose.pose.orientation).
    """
    x_cam = (px - CAM_CX) * depth / CAM_FX
    y_cam = (py - CAM_CY) * depth / CAM_FY
    world_x = drone_xyz[0] + x_cam
    world_y = drone_xyz[1] + y_cam
    return world_x, world_y


def main():
    rclpy.init()
    uav = UavControllerArduPilot("uav1")
    cam = CameraListener(uav, "/uav1/camera_down")

    if not uav.run_mission(altitude=FLIGHT_ALTITUDE):
        uav.get_logger().error("Takeoff failed.")
        uav.destroy_node()
        rclpy.shutdown()
        return

    found_objects = {}

    for wp in WAYPOINTS:
        uav.goto(*wp, hold_sec=1.0, tol=0.3)
        rclpy.spin_once(uav, timeout_sec=0.2)

        if cam.frame is None:
            uav.get_logger().warn("No camera frame yet, skipping this waypoint.")
            continue

        objs = detect_objects(cam.frame)
        drone_xyz = (uav.pose.pose.position.x, uav.pose.pose.position.y,
                     uav.pose.pose.position.z)
        depth_estimate = drone_xyz[2]   # высота дрона ~ расстояние до пола для камеры вниз

        for o in objs:
            wx, wy = pixel_to_world(*o["pixel"], depth_estimate, drone_xyz)
            key = (round(wx, 1), round(wy, 1))
            if key not in found_objects:
                found_objects[key] = {"color": o["color"], "shape": o["shape"],
                                       "x": wx, "y": wy}
                uav.get_logger().info(
                    f"Found {o['color']} {o['shape']} at ({wx:.2f}, {wy:.2f})")

    uav.goto(0.0, 0.0, FLIGHT_ALTITUDE)
    uav.land()

    uav.get_logger().info(f"Total objects found: {len(found_objects)}")
    for obj in found_objects.values():
        print(obj)

    uav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
