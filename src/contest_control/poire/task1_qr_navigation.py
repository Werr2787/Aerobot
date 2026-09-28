"""
Задание 1: Прохождение маршрута по указателям.

Сценарий:
1. Дрон взлетает.
2. Внизу он видит QR-код и берёт его номер.
3. Дрон летит к актуальным координатам этого QR из DOOR_POSITIONS.
4. Процесс повторяется до тех пор, пока не закончатся QR-указатели на полу.
5. Из всех пройденных номеров формируется число — код посадки.
6. Дрон облетает площадки, ищет площадку, на которой QR-код равен этому числу,
   и совершает посадку.

ВАЖНО:
- В словаре DOOR_POSITIONS заполните реальные координаты дверных проёмов вашей сцены.
- В словаре PAD_POSITIONS заполните реальные координаты посадочных площадок.
- Пример ниже показывает структуру и логику, готовую к запуску на реальном полигоне.
"""

import time
import rclpy
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from uav_controller_ardupilot import UavControllerArduPilot

# Рабочая высота полёта над сценой.
FLIGHT_ALTITUDE = 2.5

# Единственные актуальные координаты маршрута.
# Здесь хранятся координаты QR-кодов на полу.
# Формат: {номер_QR: (x, y, z)}
DOOR_POSITIONS = {
    3: (11, -5.0, FLIGHT_ALTITUDE),
    9: (11.0, 5.0, FLIGHT_ALTITUDE),
    7: (3.0, 5.0, FLIGHT_ALTITUDE),
    4: (2.0, 0, FLIGHT_ALTITUDE),
    # 11: (6, 0, FLIGHT_ALTITUDE),
    5: (6, 0.0, FLIGHT_ALTITUDE),
}
DOOR_PASS_OFFSET = (1.2, 0.0, 0.0)

# Координаты посадочных площадок. Каждая площадка должна иметь QR-код, который
# содержит итоговое число маршрута. Для примера ниже итоговый маршрут 1234.
PAD_POSITIONS = {
    "pad_39745": (1.2, -5.0, FLIGHT_ALTITUDE),
    "pad_4537": (1.2, -5.0,FLIGHT_ALTITUDE),
}

class QrDetector:
    """Получает изображения камеры и сохраняет распознанные QR-коды."""

    def __init__(self, node, topic):
        self.bridge = CvBridge()
        self.last_codes = []
        self.last_centers = []
        node.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = decode(frame)
        self.last_codes = [r.data.decode("utf-8").strip() for r in results]
        self.last_centers = [
            (
                (r.rect.left + r.rect.width / 2) / frame.shape[1],
                (r.rect.top + r.rect.height / 2) / frame.shape[0],
            )
            for r in results
        ]

    def clear(self):
        self.last_codes = []
        self.last_centers = []


def scan_for_code(uav, detector, timeout=5.0):
    """Ждёт появления QR-кода в кадре и возвращает первый распознанный код."""
    # Не используем QR, распознанный в предыдущей комнате.
    detector.clear()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < timeout:
        rclpy.spin_once(uav, timeout_sec=0.1)
        if detector.last_codes:
            return detector.last_codes[0]
    return None


def scan_room_for_codes(uav, qr_down, timeout=15.0):
    """Сканирует QR пола и сразу возвращает, если код есть в DOOR_POSITIONS."""
    qr_down.clear()
    found = {"floor": []}

    position = uav.pose.pose.position
    uav.set_setpoint_target(position.x, position.y, position.z)
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < timeout:
        rclpy.spin_once(uav, timeout_sec=0.1)
        for code in qr_down.last_codes:
            if code not in found["floor"]:
                found["floor"].append(code)
            qr_number = normalize_qr_value(code)
            if qr_number in DOOR_POSITIONS:
                uav.get_logger().info(
                    f"QR {qr_number} найден; сразу лечу к заданной координате."
                )
                uav.get_logger().info(
                    f"QR на полу в комнате: {found['floor']}"
                )
                return found

    uav.get_logger().info(
        f"QR на полу в комнате: {found['floor'] or 'нет'}"
    )
    return found


def normalize_qr_value(code):
    """Преобразует QR-данные к числу, если это число, иначе возвращает None."""
    if code is None:
        return None
    try:
        value = int(str(code).strip())
        return value
    except (TypeError, ValueError):
        return None


def main():
    rclpy.init()
    uav = UavControllerArduPilot("uav1")

    # qr_down — единственная камера, используемая для маршрута.
    qr_down = QrDetector(uav, "/uav1/camera_down")

    # run_mission включает GUIDED, армирует дрон и выполняет управляемый взлёт.
    if not uav.run_mission(altitude=FLIGHT_ALTITUDE):
        uav.get_logger().error("Не удалось взлететь.")
        uav.destroy_node()
        rclpy.shutdown()
        return

    uav.get_logger().info("Взлёт завершён. Начинаю сканирование всей комнаты.")
    room_codes = scan_room_for_codes(uav, qr_down, timeout=15.0)
    route = []
    door_index = 0

    # В каждой комнате QR на полу задаёт номер следующего дверного проёма.
    while True:
        floor_codes = room_codes["floor"]

        # Номер следующей точки берём только с QR на полу.
        next_door = next(
            (normalize_qr_value(code) for code in room_codes["floor"]
             if normalize_qr_value(code) in DOOR_POSITIONS),
            None,
        )
        if next_door is None:
            uav.get_logger().info(
                "QR над дверью с подходящим номером не найден. Перехожу к посадочной площадке."
            )
            break

        door_number = next_door

        route.append(door_number)
        door_index += 1
        uav.get_logger().info(f"Найден следующий проём: {door_number} (шаг {door_index})")

        target = DOOR_POSITIONS.get(door_number)
        if target is None:
            uav.get_logger().error(f"Координаты проёма {door_number} не заданы в DOOR_POSITIONS.")
            break

        # goto отвечает за полёт к координатам нужного дверного проёма.
        if not uav.goto(*target, tol=0.25):
            uav.get_logger().error("Маршрут остановлен защитой LiDAR.")
            uav.land()
            uav.destroy_node()
            rclpy.shutdown()
            return

        uav.get_logger().info("Точка QR достигнута. Сканирую следующую комнату.")
        room_codes = scan_room_for_codes(uav, qr_down, timeout=15.0)

    route_code = "".join(str(n) for n in route)
    uav.get_logger().info(f"Собранный код маршрута: {route_code}")

    if not route_code:
        uav.get_logger().warn(
            "Маршрут пустой. Лечу к QR4, затем к завершающей станции."
        )
        qr4_position = DOOR_POSITIONS[4]
        if not uav.goto(*qr4_position, tol=0.25):
            uav.land()
            uav.destroy_node()
            rclpy.shutdown()
            return

        final_position = tuple(
            coordinate + offset
            for coordinate, offset in zip(qr4_position, DOOR_PASS_OFFSET)
        )
        if not uav.goto(*final_position, tol=0.25):
            uav.land()
            uav.destroy_node()
            rclpy.shutdown()
            return
        uav.land()
        uav.destroy_node()
        rclpy.shutdown()
        return

    # После маршрута ищем площадку по итоговому коду и выполняем посадку.
    landed = False
    for pad_name, pad_pos in PAD_POSITIONS.items():
        if not uav.goto(*pad_pos, tol=0.25):
            uav.get_logger().error("Полёт к площадке остановлен защитой LiDAR.")
            uav.land()
            uav.destroy_node()
            rclpy.shutdown()
            return
        pad_code = scan_for_code(uav, qr_down, timeout=5.0)

        uav.get_logger().info(f"Площадка {pad_name}: QR = {pad_code}")
        if str(pad_code).strip() == route_code:
            uav.get_logger().info(f"Совпадение найдено: {pad_name}. Идём на посадку.")
            uav.land()
            landed = True
            break

    if not landed:
        uav.get_logger().warn(
            f"Площадка с кодом {route_code} не найдена. Сажаемся на ближайшую доступную позицию."
        )
        uav.land()

    uav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
