"""Background ROS 2 QThread client bridging Studio services/topics and PyQt5 signals."""

import math
import os
import queue
import time

from ament_index_python.packages import get_package_share_directory
from lite6_shm_client import StateShmReader, steady_now_ns
import numpy as np
import pinocchio as pin
import rclpy
from PyQt5.QtCore import QThread, pyqtSignal
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from lite6_interfaces.msg import GripperState, StudioState
from lite6_interfaces.srv import CompileProgram, GripperCommand, StudioCommand


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

        pin_model = None
        pin_data = None
        eef_frame_id = None
        try:
            urdf_path = os.path.join(
                get_package_share_directory("lite6_description"),
                "urdf",
                "lite6_arm.urdf",
            )
            pin_model = pin.buildModelFromUrdf(urdf_path)
            pin_data = pin_model.createData()
            eef_frame_id = pin_model.getFrameId("link_eef")
        except Exception as exc:
            self.ros_log.emit(f"Pinocchio FK init warning: {exc}")

        shm_reader = StateShmReader()

        compile_cli = node.create_client(CompileProgram, "/studio/compile")
        command_cli = node.create_client(StudioCommand, "/studio/command")
        gripper_cli = node.create_client(GripperCommand, "/gripper/command")

        pending = []  # list of (kind, meta, future, deadline)

        try:
            while not self._stop_requested and rclpy.ok():
                self._drain_requests(node, compile_cli, command_cli, gripper_cli, pending)
                rclpy.spin_once(node, timeout_sec=0.02)
                for _ in range(8):
                    rclpy.spin_once(node, timeout_sec=0.0)
                self._poll_pending(pending)
                self._maybe_emit_telemetry(shm_reader, pin_model, pin_data, eef_frame_id)
        finally:
            shm_reader.close()
            node.destroy_node()

    def _maybe_emit_telemetry(self, shm_reader, pin_model, pin_data, eef_frame_id):
        now = time.monotonic()
        if now - self._last_telemetry_emit < 0.05:  # 20 Hz UI throttle
            return
        self._last_telemetry_emit = now

        pod, _ = shm_reader.read_joint_state()
        if pod is None or pod.steady_stamp_ns <= 0:
            return
        if steady_now_ns() - pod.steady_stamp_ns >= 500_000_000:
            return
        q_list = [float(pod.position[i]) for i in range(6)]
        dq_list = [float(pod.velocity[i]) for i in range(6)]
        if not all(math.isfinite(x) for x in q_list + dq_list):
            return

        tcp6 = None
        if pin_model is not None and pin_data is not None and eef_frame_id is not None:
            try:
                q_arr = np.asarray(q_list, dtype=float)
                pin.forwardKinematics(pin_model, pin_data, q_arr)
                pin.updateFramePlacements(pin_model, pin_data)
                frame = pin_data.oMf[eef_frame_id]
                rpy = pin.rpy.matrixToRpy(frame.rotation)
                tcp6 = [*frame.translation.tolist(), *rpy.tolist()]
            except Exception:
                tcp6 = None

        self.telemetry_updated.emit(q_list, dq_list, tcp6)

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
