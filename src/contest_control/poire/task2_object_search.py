"""
Задание 2: Поиск объектов.

Сценарий:
  1. Взлёт, осмотр стартовой комнаты (печать найденных QR для контроля
     маршрута — это не требование самого задания 2, а удобный способ
     убедиться, что дрон реально "видит" нужные ориентиры).
  2. Систематический обход помещений: из известных точек-проёмов
     (ROOM_WAYPOINTS) всегда выбираем ещё не посещённую точку с
     МАКСИМАЛЬНЫМ X — то есть идём в крайнюю правую ещё не обследованную
     комнату, пока не обойдём все.
  3. В каждой комнате — поиск целевых объектов (цвет + форма) камерой вниз,
     расчёт координат их геометрического центра ОТНОСИТЕЛЬНО ТОЧКИ СТАРТА
     (точка взлёта = начало координат, она и так совпадает с (0,0) в
     локальной системе mavros/ArduPilot).
  4. Безопасность перемещений между комнатами обеспечивает uav.goto() —
     он уже сам обходит препятствия и центрируется в дверных проёмах по
     данным лидара, ничего дополнительно реализовывать не нужно.
  5. После обхода всех комнат — возврат на старт, посадка, печать полного
     отчёта по найденным объектам.
"""

import time
import rclpy
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from uav_controller_ardupilot import UavControllerArduPilot

FLIGHT_ALTITUDE = 2.0

# Известные точки-проёмы (из разведки сцены для Задания 1), используются
# здесь только как маршрутные точки для обхода всех комнат полигона.
ROOM_WAYPOINTS = {
    3: (-5.0, 11.0, FLIGHT_ALTITUDE),
    9: (5.0, 11.0, FLIGHT_ALTITUDE),
    7: (5.0, 3.0, FLIGHT_ALTITUDE),
    4: (0.0, 2.0, FLIGHT_ALTITUDE),
    5: (0.0, 6.0, FLIGHT_ALTITUDE),
}

START_POINT = (0.0, 0.0, FLIGHT_ALTITUDE)  # Точка старта = начало системы координат.

SURVEY_YAW_RATE = 0.3            # рад/с — скорость вращения при осмотре комнаты.
SURVEY_DURATION = 22.0           # с — полный оборот камеры при заданной скорости.

# --- Параметры детектора объектов (цвет/форма) ---
CAM_FX, CAM_FY = 554.3, 554.3    # TODO: подставьте реальные значения из /uav1/camera_down_info
CAM_CX, CAM_CY = 320.0, 240.0
MIN_CONTOUR_AREA = 300            # Отсекаем шумовые контуры меньше этой площади (px^2).
DEDUPE_RADIUS_M = 0.5             # Объекты ближе этого расстояния считаем одним и тем же.

COLOR_RANGES = {
    "red":    [((0, 100, 100), (10, 255, 255)), ((170, 100, 100), (180, 255, 255))],
    "green":  [((40, 70, 70), (80, 255, 255))],
    "blue":   [((100, 100, 100), (130, 255, 255))],
    "yellow": [((20, 100, 100), (35, 255, 255))],
}


class QrDetector:
    """Для контроля маршрута — печатаем, какие QR реально видим по пути."""

    def __init__(self, node, topic):
        self.bridge = CvBridge()
        self.codes = []
        node.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = decode(frame)
        self.codes = [r.data.decode("utf-8") for r in results]


class CameraListener:
    """Держит последний BGR-кадр камеры вниз для поиска объектов."""

    def __init__(self, node, topic):
        self.bridge = CvBridge()
        self.frame = None
        node.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def detect_objects(frame_bgr):
    """Возвращает список {color, shape, pixel:(x,y)} для объектов в кадре."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    found = []
    for color, ranges in COLOR_RANGES.items():
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < MIN_CONTOUR_AREA:
                continue
            approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
            shape = {3: "triangle", 4: "square", 5: "pentagon"}.get(len(approx), "circle")
            (cx, cy), _ = cv2.minEnclosingCircle(c)
            found.append({"color": color, "shape": shape, "pixel": (cx, cy)})
    return found


def pixel_to_world(px, py, depth, drone_xy):
    """Пересчёт пикселя камеры вниз в мировые координаты (X, Y) относительно старта."""
    x_cam = (px - CAM_CX) * depth / CAM_FX
    y_cam = (py - CAM_CY) * depth / CAM_FY
    return drone_xy[0] + x_cam, drone_xy[1] + y_cam


def math_dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def survey_and_detect(uav, qr_down, qr_front, cam_down, found_objects, scanned_qr,
                      duration=SURVEY_DURATION):
    """
    Вращается на месте, печатает новые QR (контроль маршрута) и одновременно
    ищет объекты по цвету/форме, добавляя найденные (с дедупликацией по
    расстоянию) в общий словарь found_objects.
    """
    uav.get_logger().info("Осматриваю комнату...")
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < duration:
        uav.set_velocity_target(0.0, 0.0, 0.0, SURVEY_YAW_RATE)
        rclpy.spin_once(uav, timeout_sec=0.05)

        for code in qr_down.codes:
            key = ("На полу", code)
            if key not in scanned_qr:
                x = uav.pose.pose.position.x
                y = uav.pose.pose.position.y
                z = uav.pose.pose.position.z
                scanned_qr[key] = {"location": key[0], "code": code, "x": x, "y": y, "z": z}
                line = f"[QR] {key[0]}: {code} | позиция сканирования: X={x:.2f}, Y={y:.2f}, Z={z:.2f} м"
                print(line, flush=True)
                uav.get_logger().info(line)
        for code in qr_front.codes:
            key = ("Над дверью", code)
            if key not in scanned_qr:
                x = uav.pose.pose.position.x
                y = uav.pose.pose.position.y
                z = uav.pose.pose.position.z
                scanned_qr[key] = {"location": key[0], "code": code, "x": x, "y": y, "z": z}
                line = f"[QR] {key[0]}: {code} | позиция сканирования: X={x:.2f}, Y={y:.2f}, Z={z:.2f} м"
                print(line, flush=True)
                uav.get_logger().info(line)

        if cam_down.frame is None:
            continue
        drone_xy = (uav.pose.pose.position.x, uav.pose.pose.position.y)
        depth_estimate = uav.pose.pose.position.z  # высота ~ расстояние до пола для камеры вниз

        for o in detect_objects(cam_down.frame):
            wx, wy = pixel_to_world(*o["pixel"], depth_estimate, drone_xy)
            duplicate = False
            for (ex, ey) in list(found_objects.keys()):
                if math_dist((wx, wy), (ex, ey)) < DEDUPE_RADIUS_M:
                    duplicate = True
                    break
            if not duplicate:
                found_objects[(round(wx, 2), round(wy, 2))] = {
                    "color": o["color"], "shape": o["shape"], "x": wx, "y": wy
                }
                uav.get_logger().info(
                    f"[OBJECT] {o['color']} {o['shape']} at "
                    f"(x={wx:.2f}, y={wy:.2f}) относительно старта"
                )

    uav.stop()
    uav.spin_for(0.3)


def main():
    rclpy.init()
    uav = UavControllerArduPilot("uav1")

    qr_down = QrDetector(uav, "/uav1/camera_down")
    qr_front = QrDetector(uav, "/uav1/camera")
    cam_down = CameraListener(uav, "/uav1/camera_down")

    if not uav.run_mission(altitude=FLIGHT_ALTITUDE):
        uav.get_logger().error("Взлёт не удался, завершаю миссию.")
        uav.destroy_node()
        rclpy.shutdown()
        return

    found_objects = {}
    scanned_qr = {}

    # Осмотр стартовой комнаты сразу после взлёта.
    survey_and_detect(uav, qr_down, qr_front, cam_down, found_objects, scanned_qr)

    # Систематический обход: всегда идём в ещё не посещённую точку
    # с максимальным X (крайняя правая из оставшихся).
    unvisited = dict(ROOM_WAYPOINTS)
    while unvisited:
        next_id = max(unvisited, key=lambda k: unvisited[k][0])  # максимум по X
        target = unvisited.pop(next_id)

        uav.get_logger().info(f"Лечу в комнату (точка {next_id}): {target}")
        ok = uav.goto(*target, hold_sec=0.3, tol=0.3)
        if not ok:
            uav.get_logger().error(
                f"Не удалось долететь до точки {next_id} "
                "(препятствие не обойдено или истёк таймаут). Пропускаю."
            )
            continue

        survey_and_detect(uav, qr_down, qr_front, cam_down, found_objects, scanned_qr)

    uav.get_logger().info("Все известные комнаты обследованы. Возврат на старт.")
    uav.goto(*START_POINT, tol=0.3)
    uav.land()

    print(f"=== Итоговый отчёт: отсканировано QR-кодов {len(scanned_qr)} ===", flush=True)
    if scanned_qr:
        for qr in scanned_qr.values():
            print(
                f"  {qr['location']}: {qr['code']} | позиция сканирования: "
                f"X={qr['x']:.2f}, Y={qr['y']:.2f}, Z={qr['z']:.2f} м",
                flush=True,
            )
    else:
        print("  QR-коды не обнаружены.", flush=True)

    uav.get_logger().info(f"=== Итоговый отчёт: найдено объектов {len(found_objects)} ===")
    for obj in found_objects.values():
        uav.get_logger().info(
            f"  {obj['color']:>6} {obj['shape']:>10} | "
            f"X={obj['x']:.2f} м, Y={obj['y']:.2f} м (относительно старта)"
        )
        print(obj)

    uav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()