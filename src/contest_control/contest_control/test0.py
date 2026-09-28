import rclpy
from uav_controller_ardupilot import UavControllerArduPilot


def main():
    rclpy.init()
    uav = UavControllerArduPilot("uav1")

    if uav.run_mission(altitude=2.0):
        uav.goto(x=0.0, y=0.0, z=2.0, hold_sec=10.0)   # зависание над стартом
        uav.land()
    else:
        uav.get_logger().error("Mission failed at takeoff stage.")

    uav.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()