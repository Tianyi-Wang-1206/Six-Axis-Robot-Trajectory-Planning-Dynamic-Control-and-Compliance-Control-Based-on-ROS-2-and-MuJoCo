"""Background ROS 2 QThread client bridging Studio services/topics and PyQt5 signals."""

import math
import queue
import time

import rclpy
from PyQt5.QtCore import QThread, pyqtSignal
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from lite6_interfaces.msg import GripperState, StudioState
from lite6_interfaces.srv import CompileProgram, GripperCommand, StudioCommand

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def _quat_to_rpy(x, y, z, w):
    """Convert unit quaternion [x, y, z, w] to extrinsic XYZ / intrinsic ZYX RPY [rad]."""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll, pitch, yaw]


class StudioRosWorker(QThread):
    """Runs an isolated rclpy node loop and dispatches telemetry/service results via Qt signals."""

    studio_state_updated = pyqtSignal(object)
    gripper_state_updated = pyqtSignal(object)
    telemetry_updated = pyqtSignal(object, object, object)  # q_list, dq_list, tcp6_or_none
    compile_finished = pyqtSignal(object)
    command_finished = pyqtSignal(int, bool, str)
    gripper_finished = pyqtSignal(int, bool, str)
    ros_log = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._stop_requested = False
        self._req_queue = queue.Queue()
        self._latest_q = None
        self._latest_dq = None
        self._last_telemetry_emit = 0.0

    def request_compile(self, source_code, use_current_robot_start=True, custom_start=None):
        self._req_queue.put(
            (
                "compile",
                {
                    "source_code": str(source_code),
                    "use_current": bool(use_current_robot_start),
                    "custom_start": list(custom_start) if custom_start is not None else [0.0] * 6,
                },
            )
        )

    def request_command(self, command_code, reason=""):
        self._req_queue.put(("command", {"code": int(command_code), "reason": str(reason)}))

    def request_gripper(self, action_code):
        self._req_queue.put(("gripper", {"action": int(action_code)}))

    def shutdown(self):
        self._stop_requested = True
        self.wait(3000)

    def run(self):
        node = Node("lite6_studio_hmi_node")
        state_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        node.create_subscription(
            StudioState,
            "/studio/state",
            lambda msg: self.studio_state_updated.emit(msg),
            state_qos,
        )
        node.create_subscription(
            GripperState,
            "/gripper/state",
            lambda msg: self.gripper_state_updated.emit(msg),
            10,
        )
        node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            20,
        )

        tf_buffer = Buffer()
        _tf_listener = TransformListener(tf_buffer, node, spin_thread=False)

        compile_cli = node.create_client(CompileProgram, "/studio/compile")
        command_cli = node.create_client(StudioCommand, "/studio/command")
        gripper_cli = node.create_client(GripperCommand, "/gripper/command")

        pending = []  # list of (kind, meta, future, deadline)

        try:
            while not self._stop_requested and rclpy.ok():
                self._drain_requests(node, compile_cli, command_cli, gripper_cli, pending)
                rclpy.spin_once(node, timeout_sec=0.02)
                self._poll_pending(pending)
                self._maybe_emit_telemetry(tf_buffer)
        finally:
            node.destroy_node()

    def _on_joint_state(self, msg):
        idx_map = {name: i for i, name in enumerate(msg.name)}
        q = []
        dq = []
        for jname in JOINT_NAMES:
            idx = idx_map.get(jname)
            if idx is None or idx >= len(msg.position):
                return
            q.append(float(msg.position[idx]))
            dq.append(float(msg.velocity[idx]) if idx < len(msg.velocity) else 0.0)
        self._latest_q = q
        self._latest_dq = dq

    def _maybe_emit_telemetry(self, tf_buffer):
        now = time.monotonic()
        if now - self._last_telemetry_emit < 0.05:  # 20 Hz UI throttle
            return
        self._last_telemetry_emit = now

        tcp6 = None
        try:
            tf = tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            rpy = _quat_to_rpy(r.x, r.y, r.z, r.w)
            tcp6 = [float(t.x), float(t.y), float(t.z)] + rpy
        except Exception:
            tcp6 = None

        if self._latest_q is not None or tcp6 is not None:
            self.telemetry_updated.emit(
                list(self._latest_q) if self._latest_q is not None else None,
                list(self._latest_dq) if self._latest_dq is not None else None,
                tcp6,
            )

    def _drain_requests(self, _node, compile_cli, command_cli, gripper_cli, pending):
        while True:
            try:
                kind, payload = self._req_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "compile":
                if not compile_cli.wait_for_service(timeout_sec=0.05):
                    resp = CompileProgram.Response()
                    resp.success = False
                    from lite6_interfaces.msg import CompileDiagnostic

                    d = CompileDiagnostic()
                    d.severity = CompileDiagnostic.SEVERITY_ERROR
                    d.stage = CompileDiagnostic.STAGE_SEMANTIC
                    d.line_number = 1
                    d.column_number = 1
                    d.statement_index = -1
                    d.code = "ERR_SERVICE_OFFLINE"
                    d.message = "Service /studio/compile is unavailable (is lite6_studio_planner_node running?)"
                    resp.diagnostics.append(d)
                    self.compile_finished.emit(resp)
                    continue
                req = CompileProgram.Request()
                req.source_code = payload["source_code"]
                req.use_current_robot_start = payload["use_current"]
                req.custom_start_joints = payload["custom_start"]
                fut = compile_cli.call_async(req)
                pending.append(("compile", {}, fut, time.monotonic() + 20.0))

            elif kind == "command":
                code = payload["code"]
                if not command_cli.wait_for_service(timeout_sec=0.05):
                    self.command_finished.emit(
                        code, False, "Service /studio/command is unavailable"
                    )
                    continue
                req = StudioCommand.Request()
                req.command = code
                req.reason = payload["reason"]
                fut = command_cli.call_async(req)
                pending.append(("command", {"code": code}, fut, time.monotonic() + 8.0))

            elif kind == "gripper":
                action = payload["action"]
                if not gripper_cli.wait_for_service(timeout_sec=0.05):
                    self.gripper_finished.emit(
                        action, False, "Service /gripper/command is unavailable"
                    )
                    continue
                req = GripperCommand.Request()
                req.command = action
                fut = gripper_cli.call_async(req)
                pending.append(("gripper", {"action": action}, fut, time.monotonic() + 5.0))

    def _poll_pending(self, pending):
        now = time.monotonic()
        still_pending = []
        for kind, meta, fut, deadline in pending:
            if fut.done():
                try:
                    res = fut.result()
                except Exception as exc:
                    if kind == "compile":
                        resp = CompileProgram.Response()
                        resp.success = False
                        self.compile_finished.emit(resp)
                    elif kind == "command":
                        self.command_finished.emit(meta["code"], False, str(exc))
                    else:
                        self.gripper_finished.emit(meta["action"], False, str(exc))
                    continue

                if kind == "compile":
                    self.compile_finished.emit(res)
                elif kind == "command":
                    self.command_finished.emit(meta["code"], bool(res.accepted), str(res.message))
                else:
                    self.gripper_finished.emit(meta["action"], bool(res.accepted), str(res.detail))
            elif now > deadline:
                if kind == "compile":
                    resp = CompileProgram.Response()
                    resp.success = False
                    from lite6_interfaces.msg import CompileDiagnostic

                    d = CompileDiagnostic()
                    d.severity = CompileDiagnostic.SEVERITY_ERROR
                    d.stage = CompileDiagnostic.STAGE_PLANNING
                    d.line_number = 1
                    d.column_number = 1
                    d.statement_index = -1
                    d.code = "ERR_COMPILE_TIMEOUT"
                    d.message = "Compilation service call timed out"
                    resp.diagnostics.append(d)
                    self.compile_finished.emit(resp)
                elif kind == "command":
                    self.command_finished.emit(meta["code"], False, "StudioCommand call timed out")
                else:
                    self.gripper_finished.emit(meta["action"], False, "GripperCommand call timed out")
            else:
                still_pending.append((kind, meta, fut, deadline))
        pending[:] = still_pending
