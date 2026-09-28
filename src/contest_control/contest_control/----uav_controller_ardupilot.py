import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class UaavControllerArduPilot(Node):
    """
    Контроллер для ArduPilot (режим GUIDED).
    Ключевое отличие от PX4: взлёт делается ЯВНОЙ командой через сервис
    /mavros/cmd/takeoff, а не потоком setpoint-ов. Поток setpoint-ов нужен
    уже ПОСЛЕ взлёта — для навигации по точкам.
    """

    def __init__(self, ns="uav1"):
        super().__init__("uav_controller_ap")
        self.ns = ns
        self.state = State()
        self.pose = PoseStamped()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(State, f"/{ns}/state", self._state_cb, 10)
        self.create_subscription(PoseStamped, f"/{ns}/local_position/pose",
                                self._pose_cb, qos)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, f"/{ns}/setpoint_position/local", 10)

        self.arm_cli = self.create_client(CommandBool, f"/{ns}/cmd/arming")
        self.mode_cli = self.create_client(SetMode, f"/{ns}/set_mode")
        self.takeoff_cli = self.create_client(CommandTOL, f"/{ns}/cmd/takeoff")
        self.land_cli = self.create_client(CommandTOL, f"/{ns}/cmd/land")

        # фоновый поток setpoint-ов для навигации ПОСЛЕ взлёта
        self._sp_target = None
        self._sp_timer = self.create_timer(0.05, self._sp_timer_cb)

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg

    def _sp_timer_cb(self):
        if self._sp_target is not None:
            x, y, z, yaw = self._sp_target
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            self.setpoint_pub.publish(msg)

    def set_setpoint_target(self, x, y, z, yaw=0.0):
        self._sp_target = (x, y, z, yaw)

    # ---------- вспомогательные ----------

    def spin_for(self, seconds):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_until(self, cond, timeout=15.0):
        t0 = time.time()
        while rclpy.ok() and not cond() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()

    # ---------- команды FCU ----------

    def set_mode(self, mode, timeout=10.0):
        while not self.mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_mode service...")
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None or not future.result().mode_sent:
            self.get_logger().error(f"set_mode({mode}) FAILED")
            return False
        ok = self.spin_until(lambda: self.state.mode == mode, timeout=timeout)
        if not ok:
            self.get_logger().error(f"set_mode({mode}) sent but FCU did not switch")
        return ok

    def arm(self, timeout=10.0):
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        req = CommandBool.Request()
        req.value = True
        future = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None or not future.result().success:
            self.get_logger().error(
                "ARM REJECTED. Убедитесь, что в консоли симулятора уже было "
                "'pre-arm good' перед запуском скрипта."
            )
            return False
        return self.spin_until(lambda: self.state.armed, timeout=timeout)

    def takeoff(self, altitude=2.0, timeout=15.0):
        """Явная команда взлёта — обязательна для ArduPilot."""
        while not self.takeoff_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for takeoff service...")
        req = CommandTOL.Request()
        req.altitude = float(altitude)
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        future = self.takeoff_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None or not future.result().success:
            self.get_logger().error("TAKEOFF command REJECTED by FCU")
            return False

        self.get_logger().info(f"Climbing to {altitude}m...")
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < altitude * 4 + 5:
            rclpy.spin_once(self, timeout_sec=0.1)
            if abs(self.pose.pose.position.z - altitude) < 0.2:
                self.get_logger().info("Reached target altitude.")
                break
        # активируем поток setpoint-ов на текущей точке (зависание)
        self.set_setpoint_target(
            self.pose.pose.position.x, self.pose.pose.position.y, altitude)
        return True

    def run_mission(self, altitude=2.0):
        self.get_logger().info("Switching to GUIDED...")
        if not self.set_mode("GUIDED"):
            return False
        self.get_logger().info("Arming...")
        if not self.arm():
            return False
        return self.takeoff(altitude=altitude)

    def goto(self, x, y, z, yaw=0.0, hold_sec=0.0, tol=0.3, timeout=25.0):
        self.set_setpoint_target(x, y, z, yaw)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            dx = self.pose.pose.position.x - x
            dy = self.pose.pose.position.y - y
            dz = self.pose.pose.position.z - z
            if (dx**2 + dy**2 + dz**2) ** 0.5 < tol:
                break
        if hold_sec > 0:
            self.get_logger().info(f"Holding position for {hold_sec}s...")
            self.spin_for(hold_sec)

    def land(self, timeout=15.0):
        self.get_logger().info("Landing...")
        self._sp_target = None
        while not self.land_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for land service...")
        req = CommandTOL.Request()
        future = self.land_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None or not future.result().success:
            self.get_logger().warn("LAND service call failed, falling back to set_mode(LAND)")
            self.set_mode("LAND")