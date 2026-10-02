"""
Задание 1: Прохождение маршрута по указателям.

Сценарий:
  1. Взлёт.
  2. Осмотр комнаты: дрон медленно вращается на месте и печатает в терминал
     КАЖДЫЙ новый увиденный QR-код — отдельно те, что на полу (камера вниз),
     и отдельно те, что над проёмами (камера вперёд). Это чисто
     информационная функция — помогает проверить, что код реально видит то,
     что нужно, и сверить номера с тем, что вы наблюдаете в Gazebo.
  3. Полёт по заранее известному маршруту (найден вручную через разведку
     сцены): проём 4 -> проём 5 -> проём 3 -> посадочная площадка.
  4. Обход препятствий НЕ нужно реализовывать отдельно — он уже встроен в
     uav.goto() вашего контроллера (использует лидар /uav1/scan и сам
     объезжает стены, если путь по прямой перекрыт).

Координаты DOOR_POSITIONS и ROUTE — подставьте/уточните под вашу сцену,
они уже заполнены данными, которые вы прислали.
"""

import time
import rclpy
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image

from uav_controller_ardupilot import UavControllerArduPilot

FLIGHT_ALTITUDE = 2.0

# Известные координаты QR-указателей на полу (номер проёма -> точка подлёта)
DOOR_POSITIONS = {
    3: (-5.0, 11.0, FLIGHT_ALTITUDE),
    9: (5.0, 11.0, FLIGHT_ALTITUDE),
    7: (5.0, 3.0, FLIGHT_ALTITUDE),
    4: (0.0, 2.0, FLIGHT_ALTITUDE),
    5: (0.0, 6.0, FLIGHT_ALTITUDE),
}

# Заранее известный маршрут (определён вручную по разведке сцены)
ROUTE = [4, 5, 3]

# Точка подлёта к посадочной площадке (садимся командой land(), высота
# здесь — высота ПОДЛЁТА перед посадкой, а не финальная высота касания)
LANDING_APPROACH = (-5.0, 0.0, FLIGHT_ALTITUDE)

SURVEY_YAW_RATE = 0.3        # рад/с — скорость вращения при осмотре комнаты
SURVEY_DURATION_INITIAL = 4.0  # с — время осмотра сразу после взлёта
SURVEY_DURATION_WAYPOINT = 2.0  # с — время осмотра в каждой промежуточной точке


class QrDetector:
    """Непрерывно декодирует QR-коды в кадре камеры, хранит последний набор."""

    def __init__(self, node, topic):
        self.bridge = CvBridge()
        self.codes = []  # список строк с данными всех кодов, видимых в последнем кадре
        node.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = decode(frame)
        self.codes = [r.data.decode("utf-8") for r in results]


def survey_room(uav, qr_down, qr_front, duration):
    """
    Медленно вращается на месте и печатает в терминал КАЖДЫЙ новый
    увиденный QR-код (отдельно для пола и для проёмов), без повторов.
    """
    uav.get_logger().info("Осматриваю комнату...")
    seen = set()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < duration:
        uav.set_velocity_target(0.0, 0.0, 0.0, SURVEY_YAW_RATE)
        rclpy.spin_once(uav, timeout_sec=0.05)

        for code in qr_down.codes:
            key = ("floor", code)
            if key not in seen:
                seen.add(key)
                uav.get_logger().info(f"[QR] На полу: {code}")

        for code in qr_front.codes:
            key = ("door", code)
            if key not in seen:
                seen.add(key)
                uav.get_logger().info(f"[QR] Над проёмом: {code}")

    uav.stop()
    uav.spin_for(0.3)


def main():
    rclpy.init()
    uav = UavControllerArduPilot("uav1")

    qr_down = QrDetector(uav, "/uav1/camera_down")
    qr_front = QrDetector(uav, "/uav1/camera")

    if not uav.run_mission(altitude=FLIGHT_ALTITUDE):
        uav.get_logger().error("Взлёт не удался, завершаю миссию.")
        uav.destroy_node()
        rclpy.shutdown()
        return

    survey_room(uav, qr_down, qr_front, duration=SURVEY_DURATION_INITIAL)

    for door_number in ROUTE:
        target = DOOR_POSITIONS.get(door_number)
        if target is None:
            uav.get_logger().error(
                f"Нет известных координат для проёма {door_number}, пропускаю.")
            continue

        uav.get_logger().info(f"Лечу к проёму {door_number}: {target}")
        # goto() сам останавливается и объезжает препятствие по данным лидара,
        # если прямой путь перекрыт стеной — ничего дополнительно делать не нужно.
        ok = uav.goto(*target, hold_sec=0.5, tol=0.3)
        if not ok:
            uav.get_logger().error(
                f"Не удалось долететь до проёма {door_number} "
                "(препятствие не обойдено или истёк таймаут)."
            )
            break

        survey_room(uav, qr_down, qr_front, duration=SURVEY_DURATION_WAYPOINT)

    uav.get_logger().info(f"Лечу на посадочную площадку: {LANDING_APPROACH}")
    if uav.goto(*LANDING_APPROACH, tol=0.3):
        survey_room(uav, qr_down, qr_front, duration=SURVEY_DURATION_WAYPOINT)
        uav.land()
    else:
        uav.get_logger().error(
            "Не удалось долететь до площадки, сажусь на текущей позиции.")
        uav.land()

    uav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()