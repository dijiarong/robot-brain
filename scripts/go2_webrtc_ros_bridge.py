#!/usr/bin/env python3
"""Bridge Go2 built-in LiDAR + odom (WebRTC) into ROS 2 for Nav2 smoke / phase-2.

Publishes (session-local map ≡ odom):
  - TF: map → odom (identity), odom → base_link
  - /odom (nav_msgs/Odometry)
  - /points (sensor_msgs/PointCloud2, base_link)
  - /scan  (sensor_msgs/LaserScan projection for stock Nav2 costmaps)

Optional (explicit flags only):
  - subscribe /cmd_vel → Go2 Sport Move (stream_hold)

Read-only by default (dry-run / motion off). Requires ROS 2 Jazzy + Robot-Brain
unitree-webrtc deps on the same Python (use .venv-jazzy).

Example (sensors only):
  source /opt/ros/jazzy/setup.bash
  source .venv-jazzy/bin/activate
  python scripts/go2_webrtc_ros_bridge.py

Example (Nav2 can move the dog — clear space first):
  export RDB_UNITREE_ENABLE_MOTION=true
  export RDB_UNITREE_WEBRTC_DRIVE_VIA_MOVE=true
  export RDB_UNITREE_MAX_SPEED=0.35
  python scripts/go2_webrtc_ros_bridge.py --enable-cmd-vel
"""
from __future__ import annotations

import argparse
import asyncio
import math
import os
import struct
import sys
import time
from typing import Sequence

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from config.settings import Settings
from robot_brain.actuation.unitree import UnitreeRobot
from robot_brain.actuation.unitree_webrtc import create_webrtc_transport
from robot_brain.navigation.sensors import UnitreeNavigationSensorProvider

_SPORT_MODE_LABELS = {
    0: "idle/unknown",
    1: "balance_stand",
    2: "pose",
    3: "locomotion",
}


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _points_to_laser_scan(
    points: Sequence[tuple[float, float, float]],
    *,
    z_min: float,
    z_max: float,
    range_min: float,
    range_max: float,
    bins: int,
) -> list[float]:
    ranges = [range_max] * bins
    two_pi = 2.0 * math.pi
    for x, y, z in points:
        if z < z_min or z > z_max:
            continue
        r = math.hypot(x, y)
        if r < range_min or r > range_max or not math.isfinite(r):
            continue
        ang = math.atan2(y, x)
        idx = int((ang + math.pi) / two_pi * bins) % bins
        if r < ranges[idx]:
            ranges[idx] = r
    return ranges


def _boost_min_abs(v: float, *, min_abs: float, deadzone: float) -> float:
    """Go2 ignores crawl speeds; lift non-zero cmds above gait threshold."""
    if abs(v) < deadzone:
        return 0.0
    if min_abs > 0.0 and abs(v) < min_abs:
        return math.copysign(min_abs, v)
    return v


class CmdVelGate:
    """Latest /cmd_vel with timeout → zero (Nav2 watchdog)."""

    def __init__(
        self,
        *,
        timeout_s: float,
        max_vx: float,
        max_vy: float,
        max_vyaw: float,
        min_gait_vx: float = 0.30,
        min_gait_vyaw: float = 0.25,
        cmd_deadzone: float = 0.002,
        forbid_reverse: bool = True,
    ) -> None:
        self.timeout_s = timeout_s
        self._max_vx = max_vx
        self._max_vy = max_vy
        self._max_vyaw = max_vyaw
        self._min_gait_vx = min_gait_vx
        self._min_gait_vyaw = min_gait_vyaw
        self._cmd_deadzone = cmd_deadzone
        self._forbid_reverse = forbid_reverse
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._raw_vx = 0.0
        self._stamp = 0.0
        self._count = 0

    def on_twist(self, msg: Twist) -> None:
        raw_vx = float(msg.linear.x)
        raw_vy = float(msg.linear.y)
        raw_vyaw = float(msg.angular.z)
        self._raw_vx = raw_vx
        # Go2 Nav2 smoke: never amplify reverse — tiny negative cmd becomes a hard backup.
        if self._forbid_reverse and raw_vx < 0.0:
            vx = 0.0
        else:
            vx = _boost_min_abs(
                raw_vx, min_abs=self._min_gait_vx, deadzone=self._cmd_deadzone
            )
        vy = _boost_min_abs(raw_vy, min_abs=0.0, deadzone=self._cmd_deadzone)
        vyaw = _boost_min_abs(
            raw_vyaw, min_abs=self._min_gait_vyaw, deadzone=self._cmd_deadzone
        )
        self._vx = _clamp(vx, -self._max_vx, self._max_vx)
        if self._forbid_reverse:
            self._vx = max(0.0, self._vx)
        self._vy = _clamp(vy, -self._max_vy, self._max_vy)
        self._vyaw = _clamp(vyaw, -self._max_vyaw, self._max_vyaw)
        self._stamp = time.monotonic()
        self._count += 1

    def get(self) -> tuple[float, float, float]:
        if self._stamp <= 0.0 or (time.monotonic() - self._stamp) > self.timeout_s:
            return (0.0, 0.0, 0.0)
        return (self._vx, self._vy, self._vyaw)

    @property
    def message_count(self) -> int:
        return self._count


class Go2WebRTCRosBridge(Node):
    def __init__(
        self,
        *,
        scan_bins: int,
        z_min: float,
        z_max: float,
        range_max: float,
        range_min: float = 0.35,
        enable_cmd_vel: bool = False,
        cmd_vel_gate: CmdVelGate | None = None,
        cmd_vel_topic: str = "/cmd_vel",
        publish_map_tf: bool = True,
    ) -> None:
        super().__init__("go2_webrtc_ros_bridge")
        self._scan_bins = scan_bins
        self._z_min = z_min
        self._z_max = z_max
        self._range_max = range_max
        self._range_min = range_min
        self._publish_map_tf = publish_map_tf
        self._static_tf = StaticTransformBroadcaster(self)
        self._tf = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._cloud_pub = self.create_publisher(PointCloud2, "/points", sensor_qos)
        self._static_sent = False
        self._frames = 0
        self._last_log = time.monotonic()
        self._cmd_vel_gate = cmd_vel_gate
        if enable_cmd_vel:
            if cmd_vel_gate is None:
                raise ValueError("cmd_vel_gate required when enable_cmd_vel")
            self.create_subscription(Twist, cmd_vel_topic, cmd_vel_gate.on_twist, 10)
            self.get_logger().warn(
                f"MOTION ON: subscribed {cmd_vel_topic} → Go2 Move "
                f"(timeout={cmd_vel_gate.timeout_s:.2f}s)"
            )
        else:
            self.get_logger().info(
                "Go2→ROS bridge node up (sensors only; motion off)"
            )
        if not publish_map_tf:
            self.get_logger().info(
                "map→odom TF disabled (expect slam_toolbox / AMCL to publish it)"
            )

    def ensure_static_tf(self) -> None:
        if self._static_sent:
            return
        now = self.get_clock().now().to_msg()
        transforms: list[TransformStamped] = []
        if self._publish_map_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = "map"
            tf.child_frame_id = "odom"
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        foot = TransformStamped()
        foot.header.stamp = now
        foot.header.frame_id = "base_link"
        foot.child_frame_id = "base_footprint"
        foot.transform.rotation.w = 1.0
        transforms.append(foot)
        self._static_tf.sendTransform(transforms)
        self._static_sent = True

    def publish_pose(
        self,
        *,
        x: float,
        y: float,
        yaw_deg: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vyaw: float = 0.0,
    ) -> None:
        self.ensure_static_tf()
        now = self.get_clock().now().to_msg()
        yaw = math.radians(yaw_deg)
        q = _yaw_to_quat(yaw)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = q
        self._tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vyaw
        self._odom_pub.publish(odom)

    def publish_cloud(self, points: Sequence[tuple[float, float, float]]) -> None:
        now = self.get_clock().now().to_msg()
        cloud = PointCloud2()
        cloud.header.stamp = now
        cloud.header.frame_id = "base_link"
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        buf = bytearray()
        for x, y, z in points:
            buf += struct.pack("<fff", float(x), float(y), float(z))
        cloud.data = bytes(buf)
        self._cloud_pub.publish(cloud)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / self._scan_bins
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = _points_to_laser_scan(
            points,
            z_min=self._z_min,
            z_max=self._z_max,
            range_min=self._range_min,
            range_max=self._range_max,
            bins=self._scan_bins,
        )
        self._scan_pub.publish(scan)
        self._frames += 1
        now_m = time.monotonic()
        if now_m - self._last_log >= 2.0:
            hit = sum(1 for r in scan.ranges if r < self._range_max - 1e-3)
            extra = ""
            if self._cmd_vel_gate is not None:
                vx, vy, vyaw = self._cmd_vel_gate.get()
                extra = (
                    f" cmd_vel=({vx:.2f},{vy:.2f},{vyaw:.2f})"
                    f" raw_vx={self._cmd_vel_gate._raw_vx:.3f}"
                    f" n={self._cmd_vel_gate.message_count}"
                )
            self.get_logger().info(
                f"bridge ok frames={self._frames} points={len(points)} "
                f"scan_hits={hit}/{self._scan_bins}{extra}"
            )
            self._last_log = now_m


async def _prep_locomotion(robot: UnitreeRobot, log) -> None:
    for posture, settle_s in (
        ("stand_up", 2.0),
        ("balance_stand", 1.5),
        ("free_walk", 1.5),
    ):
        await robot.set_posture(posture)
        await asyncio.sleep(settle_s)
        state = await robot.transport.read_state()
        mode = state.sport_mode
        log.info(
            f"prep {posture}: standing={state.is_standing} "
            f"sport_mode={mode} ({_SPORT_MODE_LABELS.get(mode if mode is not None else -1, '?')})"
        )
    await robot.enable_omni_teleop()
    await asyncio.sleep(0.5)
    log.info("prep done: omni teleop enabled (Move path)")


async def _cmd_vel_drive_loop(
    robot: UnitreeRobot,
    gate: CmdVelGate,
    *,
    chunk_s: float,
) -> None:
    """Only stream Move while /cmd_vel is active.

    Continuously publishing Move(0,0,0) fights the Unitree hand remote and
    makes the dog shake — stay silent when idle.
    """
    loop = asyncio.get_event_loop()
    while rclpy.ok():
        vx, vy, vyaw = gate.get()
        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(vyaw) < 1e-6:
            await asyncio.sleep(0.05)
            continue
        await robot.stream_hold(
            gate.get,
            session_deadline=loop.time() + chunk_s,
        )


async def _run(args: argparse.Namespace) -> int:
    enable_motion = bool(args.enable_cmd_vel)
    if enable_motion:
        env_ok = os.getenv("RDB_UNITREE_ENABLE_MOTION", "").lower() in (
            "1",
            "true",
            "yes",
        )
        if not env_ok:
            print(
                "ERROR: --enable-cmd-vel requires RDB_UNITREE_ENABLE_MOTION=true",
                file=sys.stderr,
            )
            return 2

    settings = Settings(
        robot_backend="unitree",
        unitree_transport="webrtc",
        navigation_backend="direct_go2",
        unitree_lidar_stream=True,
        unitree_dry_run=not enable_motion,
        unitree_enable_motion=enable_motion,
        unitree_video_relay=False,
        unitree_audio_relay=False,
    )
    if not settings.unitree_serial:
        print("ERROR: set RDB_UNITREE_SERIAL (+ remote cloud creds)", file=sys.stderr)
        return 2

    max_lin = min(settings.unitree_max_speed, settings.max_linear_speed)
    max_yaw = settings.unitree_max_yaw_speed
    gate = CmdVelGate(
        timeout_s=args.cmd_vel_timeout_s,
        max_vx=max_lin,
        max_vy=min(max_lin, args.max_vy),
        max_vyaw=max_yaw,
        min_gait_vx=args.min_gait_vx if enable_motion else 0.0,
        min_gait_vyaw=args.min_gait_vyaw if enable_motion else 0.0,
        forbid_reverse=bool(args.forbid_reverse) if enable_motion else False,
    )

    rclpy.init()
    node = Go2WebRTCRosBridge(
        scan_bins=args.scan_bins,
        z_min=args.z_min,
        z_max=args.z_max,
        range_max=args.range_max,
        range_min=args.scan_range_min,
        enable_cmd_vel=enable_motion,
        cmd_vel_gate=gate if enable_motion else None,
        cmd_vel_topic=args.cmd_vel_topic,
        publish_map_tf=bool(args.publish_map_tf),
    )
    transport = create_webrtc_transport(settings)
    robot = UnitreeRobot(transport, settings)
    sensors = UnitreeNavigationSensorProvider(
        transport,
        max_pose_age_s=settings.odom_max_age_seconds,
        max_pointcloud_age_s=max(1.0, settings.direct_nav_pointcloud_max_age_s),
        require_authoritative_odom=settings.direct_nav_require_robotodom,
    )
    drive_task: asyncio.Task | None = None
    try:
        mode = "MOTION /cmd_vel→Move" if enable_motion else "sensors-only"
        node.get_logger().info(f"connecting Go2 WebRTC ({mode})...")
        await transport.connect()
        node.get_logger().info("WebRTC connected; streaming odom + lidar → ROS 2")

        if enable_motion:
            node.get_logger().warn(
                "clear space around the dog — prep locomotion then accept /cmd_vel"
            )
            await _prep_locomotion(robot, node.get_logger())
            drive_task = asyncio.create_task(
                _cmd_vel_drive_loop(robot, gate, chunk_s=args.drive_chunk_s),
                name="cmd_vel_drive",
            )

        deadline = time.monotonic() + args.wait_ready_s
        ready_logged = False
        while rclpy.ok():
            if drive_task is not None and drive_task.done():
                exc = drive_task.exception()
                if exc is not None:
                    node.get_logger().error(f"cmd_vel drive loop died: {exc}")
                    return 4
                node.get_logger().warn("cmd_vel drive loop ended; restarting")
                drive_task = asyncio.create_task(
                    _cmd_vel_drive_loop(robot, gate, chunk_s=args.drive_chunk_s),
                    name="cmd_vel_drive",
                )

            snap = await sensors.get_snapshot()
            state = await transport.read_state()
            if snap.pose is not None and snap.pose_ready:
                vel = getattr(state, "velocity", (0.0, 0.0, 0.0)) or (0.0, 0.0, 0.0)
                node.publish_pose(
                    x=snap.pose.x_m,
                    y=snap.pose.y_m,
                    yaw_deg=snap.pose.yaw_deg,
                    vx=float(vel[0]) if len(vel) > 0 else 0.0,
                    vy=float(vel[1]) if len(vel) > 1 else 0.0,
                    vyaw=0.0,
                )
            cloud = snap.pointcloud if snap.obstacle_data_ready else None
            if cloud is None:
                cloud = transport.read_lidar_snapshot()
            if cloud is not None and cloud.points_xyz:
                frame = (cloud.frame_id or "").lower()
                if frame in {"", "base", "base_link", "unitree_lidar", "utlidar"} or (
                    snap.obstacle_data_ready and snap.obstacle_frame == "base_link"
                ):
                    node.publish_cloud(cloud.points_xyz)
            if snap.ready and not ready_logged:
                node.get_logger().info(
                    f"sensors ready pose_source={snap.pose_source} "
                    f"points={cloud.point_count if cloud else 0}"
                )
                ready_logged = True
            elif not ready_logged and time.monotonic() > deadline:
                node.get_logger().warn(
                    f"still not ready reason={snap.reason} "
                    f"pose_ready={snap.pose_ready} obstacle={snap.obstacle_data_ready}"
                )
                deadline = time.monotonic() + args.wait_ready_s

            rclpy.spin_once(node, timeout_sec=0.0)
            await asyncio.sleep(args.period_s)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if drive_task is not None:
            drive_task.cancel()
            try:
                await drive_task
            except asyncio.CancelledError:
                pass
            try:
                await robot.release_drive("bridge shutdown")
            except Exception as exc:  # noqa: BLE001
                node.get_logger().warn(f"release_drive: {exc}")
        try:
            await transport.disconnect()
        except Exception as exc:  # noqa: BLE001
            node.get_logger().warn(f"disconnect: {exc}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Go2 WebRTC lidar/odom → ROS 2 bridge")
    p.add_argument("--period-s", type=float, default=0.05)
    p.add_argument("--wait-ready-s", type=float, default=15.0)
    p.add_argument("--scan-bins", type=int, default=360)
    p.add_argument("--z-min", type=float, default=0.05)
    p.add_argument("--z-max", type=float, default=1.2)
    p.add_argument("--range-max", type=float, default=8.0)
    p.add_argument(
        "--enable-cmd-vel",
        action="store_true",
        help="Subscribe /cmd_vel and forward to Go2 Move (needs RDB_UNITREE_ENABLE_MOTION=true)",
    )
    p.add_argument("--cmd-vel-topic", type=str, default="/cmd_vel")
    p.add_argument("--cmd-vel-timeout-s", type=float, default=0.5)
    p.add_argument("--drive-chunk-s", type=float, default=2.0)
    p.add_argument(
        "--max-vy",
        type=float,
        default=0.2,
        help="Lateral speed clamp (m/s), also limited by RDB_UNITREE_MAX_SPEED",
    )
    p.add_argument(
        "--min-gait-vx",
        type=float,
        default=0.30,
        help="Boost |vx| to at least this when Nav2 sends non-zero (Go2 step threshold)",
    )
    p.add_argument(
        "--min-gait-vyaw",
        type=float,
        default=0.25,
        help="Boost |vyaw| to at least this when Nav2 sends non-zero",
    )
    p.add_argument(
        "--forbid-reverse",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Drop negative vx (default on) — avoids gait boost turning crawl reverse into backup",
    )
    p.add_argument(
        "--publish-map-tf",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Publish static map→odom (default). Disable for slam_toolbox/AMCL.",
    )
    p.add_argument(
        "--scan-range-min",
        type=float,
        default=0.35,
        help="Ignore closer LaserScan hits (filter body/legs for costmaps)",
    )
    return p.parse_args()


def main() -> None:
    raise SystemExit(asyncio.run(_run(parse_args())))


if __name__ == "__main__":
    main()
