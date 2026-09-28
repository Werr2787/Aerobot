# import rclpy

# from uav_controller import UavController


# def main():
#     rclpy.init()

#     uav = UavController("uav1")

#     # Взлёт на 2 метра
#     uav.takeoff(altitude=2.0)

#     # Полёт к точке зависания
#     uav.goto(
#         x=2.0,
#         y=0.0,
#         z=2.0,
#         hold_sec=5.0
#     )

#     # Возвращаемся на старт
#     uav.goto(
#         x=0.0,
#         y=0.0,
#         z=2.0
#     )

#     # Посадка
#     uav.land()

#     uav.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

import rclpy
from uav_controller import UavController


UAV_NAMESPACE = "uav1"
ALTITUDE = 1.5
HOVER_POINT = (2.0, 0.0)
HOVER_TIME = 10.0
START_POINT = (0.0, 0.0)


def main():
    """Run competition task 0: hover over the marked point and land."""
    rclpy.init()
    uav = UavController(UAV_NAMESPACE)
    try:
        ok = uav.takeoff(altitude=ALTITUDE)
        if not ok:
            uav.get_logger().error("Takeoff failed, aborting mission.")
            return

        uav.goto(
            x=HOVER_POINT[0],
            y=HOVER_POINT[1],
            z=ALTITUDE,
            hold_sec=HOVER_TIME,
        )
        uav.goto(x=START_POINT[0], y=START_POINT[1], z=ALTITUDE)
        uav.land()
    finally:
        uav.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()