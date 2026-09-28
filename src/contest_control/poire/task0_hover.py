import rclpy
from uav_controller_ardupilot import UavControllerArduPilot

# Точка зависания и время удержания — подставьте значения вашего полигона
HOVER_POINT = (1.0, 0.0, 2.0)
HOLD_SECONDS = 10.0


def main():
    rclpy.init()
    uav = UavControllerArduPilot("uav1")

    if uav.run_mission(altitude=HOVER_POINT[2]):
        uav.goto(*HOVER_POINT, hold_sec=HOLD_SECONDS)
        uav.goto(0.0,0.0,2.0, hold_sec=1.0)
        uav.land()
    else:
        uav.get_logger().error("Mission failed at takeoff stage.")

    uav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
