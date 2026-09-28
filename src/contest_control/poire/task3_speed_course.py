"""
Задание 3: Скоростное прохождение трассы.
Управление по скорости (не по точкам) — распознаём цветные ворота камерой
вперёд, центрируемся по курсу (P-регулятор), летим вперёд с постоянной
скоростью, притормаживаем при близком препятствии по лидару.

Перед использованием:
  - подставьте GATE_COLOR под реальный цвет маркеров ворот на вашей трассе,
  - откалибруйте FORWARD_SPEED и YAW_GAIN на низкой скорости перед тем,
    как увеличивать.
"""

import time
import rclpy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan

from uav_controller_ardupilot import UavControllerArduPilot

FLIGHT_ALTITUDE = 1.5
FORWARD_SPEED = 1.0        # м/с — начните с малого значения!
YAW_GAIN = 0.8              # коэффициент P-регулятора по курсу
MIN_OBSTACLE_DIST = 1.0     # м — минимальная дистанция по лидару перед торможением
RUN_DURATION = 30.0         # с — максимальное время прохождения трассы (защита от зависания)

GATE_COLOR_RANGE = ((40, 70, 70), (80, 255, 255))  # пример: зелёные ворота, HSV


class GateDetector:
    def __init__(self, node, topic):
        self.bridge = CvBridge()
        self.frame_width = None
        self.gate_center_x = None
        node.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.frame_width = frame.shape[1]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(GATE_COLOR_RANGE[0]), np.array(GATE_COLOR_RANGE[1]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.gate_center_x = None
            return
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 200:
            self.gate_center_x = None
            return
        (cx, _), _ = cv2.minEnclosingCircle(largest)
        self.gate_center_x = cx


class LidarListener:
    def __init__(self, node, topic):
        self.min_range = float("inf")
        node.create_subscription(LaserScan, topic, self._cb, 10)

    def _cb(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self.min_range = min(valid) if valid else float("inf")


def main():
    rclpy.init()
    uav = UavControllerArduPilot("uav1")
    gate = GateDetector(uav, "/uav1/camera")
    lidar = LidarListener(uav, "/uav1/scan")

    if not uav.run_mission(altitude=FLIGHT_ALTITUDE):
        uav.get_logger().error("Takeoff failed.")
        uav.destroy_node()
        rclpy.shutdown()
        return

    uav.get_logger().info("Starting speed run...")
    t0 = time.time()

    while rclpy.ok() and time.time() - t0 < RUN_DURATION:
        rclpy.spin_once(uav, timeout_sec=0.05)

        # Скорость вперёд — притормаживаем при близком препятствии
        speed = FORWARD_SPEED
        if lidar.min_range < MIN_OBSTACLE_DIST:
            speed *= max(0.0, lidar.min_range / MIN_OBSTACLE_DIST)

        # Курс — центрируемся на воротах, если видим их; иначе держим курс прямо
        yaw_rate = 0.0
        if gate.gate_center_x is not None and gate.frame_width:
            error_x = (gate.gate_center_x - gate.frame_width / 2) / (gate.frame_width / 2)
            yaw_rate = -YAW_GAIN * error_x

        uav.set_velocity_target(vx=speed, vy=0.0, vz=0.0, yaw_rate=yaw_rate)

    uav.get_logger().info("Speed run finished (time limit reached).")
    uav.stop()
    uav.spin_for(1.0)
    uav.land()

    uav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
