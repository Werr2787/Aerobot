import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# === ВАЖНО: поменяйте на "ardupilot", если запускаете ArduPilot-мир ===
FLIGHT_STACK = "ardupilot"

OFFBOARD_MODE = "OFFBOARD" if FLIGHT_STACK == "px4" else "GUIDED"
LAND_MODE = "AUTO.LAND" if FLIGHT_STACK == "px4" else "LAND"


class UavController(Node):
    def __init__(self, ns="uav1"):
        super().__init__("uav_controller")
        self.ns = ns
        self.state = State()
        self.pose = PoseStamped()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(State, f"/{ns}/mavros/state", self._state_cb, 10)
        self.create_subscription(PoseStamped, f"/{ns}/mavros/local_position/pose",
                                  self._pose_cb, qos)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, f"/{ns}/mavros/setpoint_position/local", 10)

        self.arm_cli = self.create_client(CommandBool, f"/{ns}/mavros/cmd/arming")
        self.mode_cli = self.create_client(SetMode, f"/{ns}/mavros/set_mode")

        # --- ФОНОВЫЙ поток setpoint-ов. Публикуется на таймере 20 Гц НЕЗАВИСИМО
        # от того, чем занят основной код (в том числе во время ожидания
        # ответа от set_mode/arming сервисов). Это критично для PX4: если поток
        # setpoint-ов прерывается более чем на ~500мс, PX4 сам выходит из OFFBOARD.
        self._sp_target = None  # (x, y, z, yaw) или None
        self._sp_timer = self.create_timer(0.05, self._sp_timer_cb)

    def _state_cb(self, msg):
        self.state = msg

    def _pose_cb(self, msg):
        self.pose = msg

    def _sp_timer_cb(self):
        if self._sp_target is not None:
            x, y, z, yaw = self._sp_target
            self._publish_setpoint_now(x, y, z, yaw)

    # ---------- вспомогательные ----------

    def spin_for(self, seconds):
        """Просто крутит rclpy заданное время, не блокируя фоновый таймер setpoint-ов."""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_until(self, cond, timeout=15.0):
        t0 = time.time()
        while rclpy.ok() and not cond() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return cond()

    def _publish_setpoint_now(self, x, y, z, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        self.setpoint_pub.publish(msg)

    def set_setpoint_target(self, x, y, z, yaw=0.0):
        """Задать текущую цель — таймер сам будет непрерывно слать её на фоне."""
        self._sp_target = (x, y, z, yaw)

    def wait_for_position_estimate(self, min_stable_sec=5.0, timeout=40.0):
        self.get_logger().info("Waiting for position estimate to stabilize...")
        t_start = time.time()
        stable_since = None
        while rclpy.ok() and time.time() - t_start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose.pose.position
            has_pose = (p.x != 0.0 or p.y != 0.0 or p.z != 0.0)
            if has_pose:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since > min_stable_sec:
                    self.get_logger().info("Position estimate looks stable.")
                    return True
            else:
                stable_since = None
        self.get_logger().warn("Timed out waiting for stable position estimate.")
        return False

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
                "ARM REJECTED by FCU. Check preflight checks "
                "(QGroundControl -> Vehicle status, or /diagnostics topic)."
            )
            return False
        return self.spin_until(lambda: self.state.armed, timeout=timeout)

    # ---------- полёт ----------

    def takeoff(self, altitude=1.5, x=0.0, y=0.0, yaw=0.0, settle_sec=3.0):
        if not self.wait_for_position_estimate():
            self.get_logger().error("No stable position estimate, aborting takeoff.")
            return False

        # Включаем фоновую публикацию цели ДО смены режима — таймер уже
        # работает непрерывно на 20 Гц и не остановится во время set_mode/arm.
        self.get_logger().info("Streaming setpoints (background timer) before OFFBOARD...")
        self.set_setpoint_target(x, y, altitude, yaw)
        self.spin_for(2.0)  # даём накопиться потоку минимум 2 сек (40 setpoint-ов на 20Гц)

        if not self.set_mode(OFFBOARD_MODE):
            return False

        if not self.arm():
            return False

        self.get_logger().info(f"Armed. Climbing to altitude={altitude}m...")
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < settle_sec + altitude * 3:
            rclpy.spin_once(self, timeout_sec=0.05)
            if abs(self.pose.pose.position.z - altitude) < 0.15:
                self.get_logger().info("Reached target altitude.")
                break
        return True

    def goto(self, x, y, z, yaw=0.0, hold_sec=0.0, tol=0.2, timeout=20.0):
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

    def land(self):
        self.get_logger().info("Landing...")
        self.set_mode(LAND_MODE)
        self._sp_target = None  # больше не мешаем автопилоту своими setpoint-ами