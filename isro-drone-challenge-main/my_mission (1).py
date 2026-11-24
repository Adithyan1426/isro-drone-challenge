#!/usr/bin/env python3
"""
ArduPilot GPS-denied hover mission — MAVROS + Optical Flow + LiDAR
===================================================================
Tested with: ArduCopter 4.x, MAVROS 2.x, ROS 2 Humble/Iron

ArduPilot parameters you MUST set before flying:
  GPS_TYPE        = 0   (disable GPS)
  EK3_SRC1_POSXY  = 5   (optical flow for XY position)
  EK3_SRC1_VELXY  = 5   (optical flow for XY velocity)
  EK3_SRC1_POSZ   = 6   (rangefinder for Z / altitude)
  EK3_SRC1_YAW    = 1   (gyro compass)
  EK3_GPS_TYPE    = 3   (disable EKF GPS fusion)
  FLOW_TYPE       = <your sensor driver number>
  RNGFND1_TYPE    = <your LiDAR driver number>
  RNGFND1_MAX_CM  = <your LiDAR max range in cm>
  RNGFND1_MIN_CM  = <your LiDAR min range in cm>
  RNGFND1_ORIENT  = 25  (downward facing)
  LAND_SPEED      = 30  (cm/s descent — tune for your airframe)

Flight mode: GUIDED   (accepts position setpoints + CommandTOL without GPS)
Land  mode:  LAND     (ArduPilot built-in — uses LiDAR for flare)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
import time

# ── mission parameters ────────────────────────────────────────────────────────
HOVER_ALTITUDE     = 5.0    # metres AGL (relative to EKF3 home)
HOVER_DURATION     = 300.0  # seconds — 5 minutes
SETPOINT_HZ        = 10.0   # Hz — publish rate for position hold
TAKEOFF_ALTITUDE   = 5.0    # metres passed to CommandTOL
TAKEOFF_TIMEOUT    = 25.0   # seconds allowed for climb
ALTITUDE_EPSILON   = 0.25   # metres — tolerance for "reached altitude"
LAND_WAIT_TIMEOUT  = 90.0   # seconds to wait for auto-disarm after LAND


class MyMission(Node):
    def __init__(self):
        super().__init__('my_mission')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── state ─────────────────────────────────────────────────────────────
        self.current_state = State()
        self.current_pose  = PoseStamped()
        self.pose_received = False

        # ── subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            State, '/mavros/state',
            self._state_cb, qos
        )
        # EKF3 fuses optical flow (XY) + LiDAR (Z) and publishes here
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._pose_cb, qos
        )

        # ── publishers ────────────────────────────────────────────────────────
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )

        # ── service clients ───────────────────────────────────────────────────
        self.arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode,     '/mavros/set_mode')
        self.takeoff_client  = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _state_cb(self, msg: State):
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose  = msg
        self.pose_received = True

    # ── private helpers ───────────────────────────────────────────────────────
    def _spin(self, secs: float):
        rclpy.spin_once(self, timeout_sec=secs)

    def _z(self) -> float:
        return self.current_pose.pose.position.z

    def _wait_for_fcu(self, timeout: float = 30.0) -> bool:
        self.get_logger().info('Waiting for FCU connection …')
        deadline = time.time() + timeout
        while not self.current_state.connected:
            self._spin(0.1)
            if time.time() > deadline:
                self.get_logger().error('FCU connection timed out')
                return False
        self.get_logger().info('FCU connected')
        return True

    def _wait_for_local_position(self, timeout: float = 15.0) -> bool:
        """
        Wait for EKF3 to publish a valid local position.
        If this times out, optical flow or LiDAR is not being fused —
        check EK3_SRC1_POSXY, EK3_SRC1_POSZ, FLOW_TYPE, RNGFND1_TYPE.
        """
        self.get_logger().info(
            'Waiting for EKF3 local position (optical flow + LiDAR) …'
        )
        deadline = time.time() + timeout
        while not self.pose_received:
            self._spin(0.1)
            if time.time() > deadline:
                self.get_logger().error(
                    'No local_position/pose received.\n'
                    '  Check: EK3_SRC1_POSXY=5, EK3_SRC1_POSZ=6,\n'
                    '         FLOW_TYPE, RNGFND1_TYPE, sensor wiring.'
                )
                return False
        self.get_logger().info(
            f'EKF3 position OK — '
            f'x={self.current_pose.pose.position.x:.2f}  '
            f'y={self.current_pose.pose.position.y:.2f}  '
            f'z={self._z():.2f}'
        )
        return True

    def _wait_for_services(self, timeout: float = 20.0) -> bool:
        for client in (self.arming_client, self.set_mode_client,
                       self.takeoff_client):
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(
                    f'MAVROS service unavailable: {client.srv_name}'
                )
                return False
        return True

    def _set_mode(self, mode: str, retries: int = 5) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        for attempt in range(1, retries + 1):
            future = self.set_mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.result() and future.result().mode_sent:
                self.get_logger().info(f'Mode → {mode}')
                return True
            self.get_logger().warn(
                f'Set mode attempt {attempt}/{retries} failed, retrying …'
            )
            time.sleep(0.5)
        self.get_logger().error(f'Could not set mode: {mode}')
        return False

    def _arm(self, retries: int = 5) -> bool:
        req = CommandBool.Request()
        req.value = True
        for attempt in range(1, retries + 1):
            future = self.arming_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            if future.result() and future.result().success:
                self.get_logger().info('Armed')
                return True
            self.get_logger().warn(
                f'Arm attempt {attempt}/{retries} failed — '
                'check GCS for pre-arm errors'
            )
            time.sleep(0.5)
        self.get_logger().error(
            'Arming failed. Common causes:\n'
            '  - EKF3 not healthy (check optical flow & LiDAR)\n'
            '  - ARMING_CHECK items not satisfied\n'
            '  - Not in GUIDED mode'
        )
        return False

    def _publish_setpoint(self, x: float, y: float, z: float):
        sp = PoseStamped()
        sp.header.stamp    = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        sp.pose.orientation.w = 1.0  # hold yaw — no rotation
        self.setpoint_pub.publish(sp)

    def _hold_position(self, x: float, y: float, z: float,
                       duration: float, hz: float = SETPOINT_HZ):
        """Publish position setpoints at hz for duration seconds."""
        interval  = 1.0 / hz
        end_time  = time.time() + duration
        log_every = int(hz * 30)   # log every 30 s
        count     = 0
        while time.time() < end_time:
            self._publish_setpoint(x, y, z)
            self._spin(interval)
            count += 1
            if count % log_every == 0:
                remaining = end_time - time.time()
                self.get_logger().info(
                    f'Hovering — z: {self._z():.2f}/{z:.1f} m  '
                    f'remaining: {remaining:.0f} s'
                )

    def _wait_for_altitude(self, target_z: float,
                           timeout: float = TAKEOFF_TIMEOUT) -> bool:
        """Block until EKF3 reports the vehicle is at target altitude."""
        self.get_logger().info(
            f'Climbing to {target_z:.1f} m  '
            f'(currently at {self._z():.2f} m) …'
        )
        deadline = time.time() + timeout
        while abs(self._z() - target_z) > ALTITUDE_EPSILON:
            self._spin(0.3)
            self.get_logger().info(
                f'  altitude: {self._z():.2f} m',
                throttle_duration_sec=3
            )
            if time.time() > deadline:
                self.get_logger().warn(
                    f'Climb timeout — at {self._z():.2f} m, '
                    f'target {target_z:.1f} m. Proceeding.'
                )
                return False
        self.get_logger().info(f'Reached {self._z():.2f} m')
        return True

    # ── mission entry point ───────────────────────────────────────────────────
    def run_mission(self):

        # 1. FCU handshake
        if not self._wait_for_fcu():
            return

        # 2. MAVROS services ready
        if not self._wait_for_services():
            return

        # 3. Confirm EKF3 local position is healthy
        #    optical flow → XY,  LiDAR → Z
        if not self._wait_for_local_position():
            return

        # 4. Record home position (ground level, before any movement)
        home_x = self.current_pose.pose.position.x
        home_y = self.current_pose.pose.position.y
        self.get_logger().info(
            f'Home XY locked: ({home_x:.3f}, {home_y:.3f})'
        )

        # 5. GUIDED mode — ArduPilot GUIDED accepts local setpoints
        #    and CommandTOL using EKF3 local position (no GPS required)
        if not self._set_mode('GUIDED'):
            return

        # 6. Arm motors
        if not self._arm():
            return

        # 7. Takeoff — CommandTOL works in ArduPilot GUIDED without GPS
        #    as long as EKF3 has a valid local position from optical flow + LiDAR
        self.get_logger().info(f'Takeoff to {TAKEOFF_ALTITUDE} m …')
        tol              = CommandTOL.Request()
        tol.altitude     = TAKEOFF_ALTITUDE  # metres AGL relative to EKF3 home
        tol.latitude     = 0.0               # 0.0 = use current local position
        tol.longitude    = 0.0
        tol.min_pitch    = 0.0
        tol.yaw          = 0.0               # hold current heading

        future = self.takeoff_client.call_async(tol)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not (future.result() and future.result().success):
            self.get_logger().error(
                'CommandTOL rejected.\n'
                '  Verify: GUIDED mode active, EKF3 healthy,\n'
                '          ARMING_CHECK satisfied, motors armed.'
            )
            return

        # 8. Wait for vehicle to reach hover altitude
        self._wait_for_altitude(HOVER_ALTITUDE)

        # 9. Hover at home XY, HOVER_ALTITUDE for HOVER_DURATION seconds
        #    Setpoints keep GUIDED active and correct any position drift.
        #    Optical flow corrects XY drift; LiDAR corrects Z drift.
        self.get_logger().info(
            f'Holding position for {HOVER_DURATION:.0f} s …'
        )
        self._hold_position(
            home_x, home_y, HOVER_ALTITUDE,
            duration=HOVER_DURATION,
        )

        # 10. Re-assert exact home XY for 3 s before descent
        #     Corrects any residual drift so landing spot = takeoff spot
        self.get_logger().info('Re-centring over home XY …')
        self._hold_position(home_x, home_y, HOVER_ALTITUDE, duration=3.0)

        # 11. Switch to LAND
        #     ArduPilot LAND descends at LAND_SPEED (param, default 50 cm/s),
        #     uses LiDAR for flare detection, and auto-disarms on touchdown.
        self.get_logger().info('Switching to LAND mode …')
        if not self._set_mode('LAND'):
            self.get_logger().error(
                'LAND mode failed — disarm via GCS immediately!'
            )
            return

        # 12. Wait for auto-disarm (ArduPilot disarms automatically after landing)
        self.get_logger().info('Landing … waiting for auto-disarm')
        deadline = time.time() + LAND_WAIT_TIMEOUT
        while self.current_state.armed:
            self._spin(0.5)
            self.get_logger().info(
                f'  altitude: {self._z():.2f} m',
                throttle_duration_sec=5
            )
            if time.time() > deadline:
                self.get_logger().warn(
                    'Auto-disarm timeout — check vehicle status in GCS.'
                )
                break

        self.get_logger().info('Mission complete.')


def main():
    rclpy.init()
    node = MyMission()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        node.get_logger().info(
            'Interrupted — switch to LAND mode via GCS if still airborne!'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
