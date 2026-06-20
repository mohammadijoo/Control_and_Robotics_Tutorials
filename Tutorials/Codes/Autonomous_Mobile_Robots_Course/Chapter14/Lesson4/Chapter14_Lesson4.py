"""
Chapter14_Lesson4.py
Recovery Behaviors and Fault Handling — ROS 2 / Nav2-oriented supervisor (example)

Notes:
- This script is a *teaching* example. Topic/service names can differ by robot/nav2 config.
- It demonstrates: progress monitoring, oscillation detection, costmap clearing, simple spin/backup,
  and escalation to safe-stop.

Dependencies (typical):
  sudo apt install ros-<distro>-nav2-msgs ros-<distro>-geometry-msgs ros-<distro>-nav-msgs
  pip install --user numpy
"""

import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Nav2 message/service types (names may vary by distro)
from nav2_msgs.srv import ClearEntireCostmap


@dataclass
class MonitorParams:
    # Progress monitor
    window_s: float = 10.0          # seconds
    min_progress_m: float = 0.25    # required decrease in distance-to-goal over window
    # Oscillation monitor
    osc_window_s: float = 6.0
    osc_sign_flips: int = 6         # number of sign flips in cmd_vel over window
    v_eps: float = 0.03             # treat |v| < v_eps as zero
    # Localization uncertainty (AMCL covariance trace threshold)
    cov_trace_max: float = 0.35
    # Recovery actions
    spin_time_s: float = 3.0
    spin_wz: float = 0.8
    backup_time_s: float = 2.0
    backup_vx: float = -0.12
    # Escalation
    max_recovery_attempts: int = 3
    # Topics/services (common defaults)
    cmd_vel_topic: str = "/cmd_vel"
    odom_topic: str = "/odom"
    amcl_topic: str = "/amcl_pose"
    local_clear_srv: str = "/local_costmap/clear_entirely_local_costmap"
    global_clear_srv: str = "/global_costmap/clear_entirely_global_costmap"


class RecoverySupervisor(Node):
    def __init__(self):
        super().__init__("recovery_supervisor")

        # Parameters (could be declared as ROS params; kept simple for teaching)
        self.p = MonitorParams()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub_cmd = self.create_publisher(Twist, self.p.cmd_vel_topic, 10)
        self.sub_odom = self.create_subscription(Odometry, self.p.odom_topic, self.on_odom, qos)
        self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, self.p.amcl_topic, self.on_amcl, 10)
        self.sub_cmd = self.create_subscription(Twist, self.p.cmd_vel_topic, self.on_cmd, 10)

        self.cli_local = self.create_client(ClearEntireCostmap, self.p.local_clear_srv)
        self.cli_global = self.create_client(ClearEntireCostmap, self.p.global_clear_srv)

        # State buffers
        self.goal_xy = None  # (gx, gy) set externally (e.g., from NavigateToPose goal); for demo we keep None
        self.last_xy = None
        self.last_cov_trace = None

        self.progress_hist = []  # list of (t, dist_to_goal)
        self.cmd_hist = []       # list of (t, vx, wz)
        self.recovery_attempts = 0
        self.in_recovery = False

        # Periodic check loop
        self.timer = self.create_timer(0.2, self.tick)

        self.get_logger().info("RecoverySupervisor started. Set goal_xy in code or adapt to listen to Nav2 goals.")

    # ------------------------- Callbacks -------------------------

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.last_xy = (x, y)

        if self.goal_xy is not None:
            d = float(np.hypot(self.goal_xy[0] - x, self.goal_xy[1] - y))
            t = self.now_s()
            self.progress_hist.append((t, d))
            self.progress_hist = self.trim_by_time(self.progress_hist, self.p.window_s)

    def on_amcl(self, msg: PoseWithCovarianceStamped):
        cov = np.array(msg.pose.covariance, dtype=float).reshape(6, 6)
        self.last_cov_trace = float(np.trace(cov[:3, :3]))  # x,y,yaw block approx

    def on_cmd(self, msg: Twist):
        t = self.now_s()
        self.cmd_hist.append((t, float(msg.linear.x), float(msg.angular.z)))
        self.cmd_hist = self.trim_by_time(self.cmd_hist, self.p.osc_window_s)

    # ------------------------- Helpers -------------------------

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def trim_by_time(hist, window_s):
        if not hist:
            return hist
        t_latest = hist[-1][0]
        t_min = t_latest - window_s
        i = 0
        while i < len(hist) and hist[i][0] < t_min:
            i += 1
        return hist[i:]

    def progress_ok(self) -> bool:
        """Return True if sufficient progress toward goal occurred in the window."""
        if self.goal_xy is None or len(self.progress_hist) < 2:
            return True  # not enough info; don't fault
        d_old = self.progress_hist[0][1]
        d_new = self.progress_hist[-1][1]
        return (d_old - d_new) >= self.p.min_progress_m

    def oscillating(self) -> bool:
        """Detect oscillation by counting sign flips of linear-x or angular-z commands."""
        if len(self.cmd_hist) < 3:
            return False

        def sign_eps(v):
            if abs(v) < self.p.v_eps:
                return 0
            return 1 if v > 0 else -1

        sx = [sign_eps(vx) for _, vx, _ in self.cmd_hist]
        sz = [sign_eps(wz) for _, _, wz in self.cmd_hist]

        flips_x = sum(1 for i in range(1, len(sx)) if sx[i] != 0 and sx[i-1] != 0 and sx[i] != sx[i-1])
        flips_z = sum(1 for i in range(1, len(sz)) if sz[i] != 0 and sz[i-1] != 0 and sz[i] != sz[i-1])

        return (flips_x + flips_z) >= self.p.osc_sign_flips

    def localization_bad(self) -> bool:
        if self.last_cov_trace is None:
            return False
        return self.last_cov_trace > self.p.cov_trace_max

    # ------------------------- Recovery actions -------------------------

    def call_clear_costmaps(self):
        """Best-effort: clear local then global costmap."""
        for cli, name in [(self.cli_local, "local"), (self.cli_global, "global")]:
            if not cli.service_is_ready():
                self.get_logger().warn(f"Clear {name} costmap service not ready: {cli.srv_name}")
                continue
            req = ClearEntireCostmap.Request()
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            self.get_logger().info(f"Clear {name} costmap result: {fut.result() is not None}")

    def publish_for(self, vx: float, wz: float, duration_s: float):
        """Open-loop velocity command for a limited time."""
        t_end = time.time() + duration_s
        msg = Twist()
        while rclpy.ok() and time.time() < t_end:
            msg.linear.x = float(vx)
            msg.angular.z = float(wz)
            self.pub_cmd.publish(msg)
            time.sleep(0.05)
        # stop
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)

    def do_recovery_cycle(self, reason: str):
        self.in_recovery = True
        self.recovery_attempts += 1

        self.get_logger().warn(f"RECOVERY #{self.recovery_attempts}: reason={reason}")

        # 1) Clear costmaps (handles transient obstacles / stale inflation)
        self.call_clear_costmaps()

        # 2) Spin in place (improves sensor coverage / localization)
        self.publish_for(0.0, self.p.spin_wz, self.p.spin_time_s)

        # 3) Backup (escape tight corners / local minima near obstacles)
        self.publish_for(self.p.backup_vx, 0.0, self.p.backup_time_s)

        self.in_recovery = False

    def safe_stop(self, reason: str):
        self.get_logger().error(f"SAFE STOP: {reason}")
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        # publish a few times
        for _ in range(10):
            self.pub_cmd.publish(msg)
            time.sleep(0.05)

    # ------------------------- Main tick -------------------------

    def tick(self):
        if self.in_recovery:
            return

        if self.recovery_attempts >= self.p.max_recovery_attempts:
            self.safe_stop("exceeded max recovery attempts")
            rclpy.shutdown()
            return

        # Fault conditions (simple; extend with more residuals)
        if self.localization_bad():
            self.do_recovery_cycle("localization covariance too high")
            return

        if self.goal_xy is not None:
            if (not self.progress_ok()) or self.oscillating():
                why = "no progress" if not self.progress_ok() else "oscillation"
                self.do_recovery_cycle(why)
                return


def main():
    rclpy.init()
    node = RecoverySupervisor()

    # DEMO: set a fixed goal (replace with nav2 action goal listener)
    node.goal_xy = (5.0, 0.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
