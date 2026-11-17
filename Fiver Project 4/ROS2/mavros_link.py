"""
Shared ROS 2 (rclpy) connection to MAVROS 2: telemetry + checked FCU commands.

Used by the ROS 2 versions of mission.py, images.py and the check scripts.
The ROS executor spins in a background thread, so the scripts can be written
as simple step-by-step code. Ctrl+C raises KeyboardInterrupt while ROS keeps
working, so a script can still send RTL / LAND / disarm before exiting.
"""
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64
from mavros_msgs.msg import State, WaypointReached, StatusText
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear, StreamRate
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState


class LinkError(Exception):
    pass


class MavrosLink:
    def __init__(self, node_name, ns="/mavros"):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node(node_name)
        self.ns = ns.rstrip("/")

        # telemetry
        self.state = None
        self.state_time = 0.0
        self.battery = None              # percent, None = not reported
        self.battery_voltage = None
        self.gps_status = NavSatStatus.STATUS_NO_FIX
        self.lat = self.lon = self.amsl = None
        self.rel_alt = None
        self.last_reached = -1
        self.fcu_text = ""               # last pre-arm / warning text from ArduPilot

        # Best-effort QoS is compatible with both reliable and best-effort publishers
        qos = qos_profile_sensor_data
        sub = self.node.create_subscription
        sub(State, f"{self.ns}/state", self._state, qos)
        sub(BatteryState, f"{self.ns}/battery", self._battery, qos)
        sub(NavSatFix, f"{self.ns}/global_position/global", self._gps, qos)
        sub(Float64, f"{self.ns}/global_position/rel_alt", self._rel_alt, qos)
        sub(WaypointReached, f"{self.ns}/mission/reached", self._reached, 10)
        sub(StatusText, f"{self.ns}/statustext/recv", self._statustext, qos)

        cli = self.node.create_client
        self._arm = cli(CommandBool, f"{self.ns}/cmd/arming")
        self._mode = cli(SetMode, f"{self.ns}/set_mode")
        self._takeoff = cli(CommandTOL, f"{self.ns}/cmd/takeoff")
        self._clear = cli(WaypointClear, f"{self.ns}/mission/clear")
        self._push = cli(WaypointPush, f"{self.ns}/mission/push")
        self._stream = cli(StreamRate, f"{self.ns}/set_stream_rate")

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        threading.Thread(target=self._keep_streams, daemon=True).start()

    def _keep_streams(self):
        """ArduPilot only sends GPS/battery/altitude data when a ground station asks
        for it. Ask (and re-ask while data is missing, e.g. after an FCU reboot)."""
        while rclpy.ok():
            if self.connected and (self.lat is None or self.rel_alt is None or self.battery_voltage is None):
                if self._stream.wait_for_service(timeout_sec=2.0):
                    self._call(self._stream, StreamRate.Request(stream_id=0, message_rate=10, on_off=True))
            time.sleep(3.0)

    # ---------------- logging ----------------
    def info(self, msg):
        self.node.get_logger().info(msg)

    def warn(self, msg):
        self.node.get_logger().warning(msg)

    def error(self, msg):
        self.node.get_logger().error(msg)

    # ---------------- callbacks ----------------
    def _state(self, msg):
        self.state = msg
        self.state_time = time.time()

    def _battery(self, msg):
        self.battery_voltage = msg.voltage
        if not math.isnan(msg.percentage) and msg.percentage >= 0:
            self.battery = msg.percentage * 100.0

    def _gps(self, msg):
        # NavSatStatus: -1 NO_FIX, 0 FIX, 1 SBAS_FIX, 2 GBAS_FIX
        self.gps_status = msg.status.status
        if not (math.isnan(msg.latitude) or math.isnan(msg.longitude)):
            self.lat, self.lon, self.amsl = msg.latitude, msg.longitude, msg.altitude

    def _rel_alt(self, msg):
        self.rel_alt = msg.data

    def _reached(self, msg):
        self.last_reached = max(self.last_reached, msg.wp_seq)
        self.info(f"Waypoint reached: {msg.wp_seq}")

    def _statustext(self, msg):
        # severity <= 4 = WARNING or worse; "PreArm:" / "Arm:" texts explain arming refusals
        if msg.severity <= 4 or "arm" in msg.text.lower():
            self.fcu_text = msg.text

    # ---------------- state helpers ----------------
    @property
    def connected(self):
        return self.state is not None and self.state.connected and time.time() - self.state_time < 5.0

    @property
    def armed(self):
        return self.state is not None and self.state.armed

    @property
    def mode(self):
        return self.state.mode if self.state is not None else ""

    @property
    def gps_ok(self):
        return self.gps_status >= NavSatStatus.STATUS_FIX and self.lat is not None

    @staticmethod
    def wait_until(predicate, timeout, hz=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(1.0 / hz)
        return predicate()

    # ---------------- services ----------------
    def wait_for_services(self, timeout=30.0):
        for c in (self._arm, self._mode, self._takeoff, self._clear, self._push):
            if not c.wait_for_service(timeout_sec=timeout):
                raise LinkError(f"MAVROS service {c.srv_name} not available - is mavros running?")

    def _call(self, client, request, timeout=15.0):
        future = client.call_async(request)
        end = time.time() + timeout
        while not future.done() and time.time() < end:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            self.error(f"{client.srv_name}: no answer within {timeout:.0f} s")
            return None
        if future.exception() is not None:
            self.error(f"{client.srv_name} failed: {future.exception()}")
            return None
        return future.result()

    def set_mode(self, mode, timeout=10.0):
        resp = self._call(self._mode, SetMode.Request(custom_mode=mode))
        if resp is None or not resp.mode_sent:
            self.error(f"FCU rejected mode {mode}")
            return False
        if not self.wait_until(lambda: self.mode == mode, timeout):
            self.error(f"Mode did not change to {mode} (still {self.mode})")
            return False
        return True

    def arm(self, value, timeout=10.0):
        resp = self._call(self._arm, CommandBool.Request(value=value))
        if resp is None or not resp.success:
            self.error(f"FCU rejected {'arming' if value else 'disarming'} "
                       f"(result {getattr(resp, 'result', '?')}) - check pre-arm messages")
            return False
        return self.wait_until(lambda: self.armed == value, timeout)

    def arm_for_flight(self, timeout=60.0):
        """Arm, retrying while ArduPilot's pre-arm checks settle (IMU / EKF / GPS need
        30-60 s after power-on). A refused arm request changes nothing, so retrying
        is safe. ArduPilot's own reason ("PreArm: ...") is shown each time."""
        end = time.time() + timeout
        attempt = 0
        while True:
            attempt += 1
            self.fcu_text = ""
            resp = self._call(self._arm, CommandBool.Request(value=True))
            if resp is not None and resp.success and self.wait_until(lambda: self.armed, 10):
                return True
            time.sleep(1.0)  # give the FCU's explanation time to arrive
            reason = self.fcu_text or f"result {getattr(resp, 'result', '?')}"
            if time.time() >= end:
                self.error(f"Arming refused: {reason}")
                return False
            self.warn(f"Arming refused ({reason}) - retry {attempt} (up to {timeout:.0f} s)")
            time.sleep(4.0)

    def takeoff(self, altitude):
        req = CommandTOL.Request(min_pitch=0.0, yaw=float("nan"), latitude=float(self.lat or 0.0),
                                 longitude=float(self.lon or 0.0), altitude=float(altitude))
        resp = self._call(self._takeoff, req)
        if resp is None or not resp.success:
            self.error(f"FCU rejected takeoff (result {getattr(resp, 'result', '?')})")
            return False
        return True

    def upload(self, waypoints):
        resp = self._call(self._clear, WaypointClear.Request())
        if resp is None or not resp.success:
            self.error("Could not clear the old mission")
            return False
        resp = self._call(self._push, WaypointPush.Request(start_index=0, waypoints=waypoints), timeout=60)
        if resp is None or not resp.success or resp.wp_transfered != len(waypoints):
            self.error(f"Mission upload failed ({getattr(resp, 'wp_transfered', 0)}/{len(waypoints)} items)")
            return False
        return True

    def shutdown(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
