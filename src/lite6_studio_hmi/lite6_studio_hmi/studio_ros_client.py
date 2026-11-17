"""Background ROS 2 QThread client bridging Studio services/topics and PyQt5 signals."""

import math
import os
import queue
import threading
import time

from ament_index_python.packages import get_package_share_directory
from lite6_shm_client import StateShmReader, steady_now_ns
import numpy as np
import pinocchio as pin
import rclpy
from PyQt5.QtCore import QThread, pyqtSignal
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from lite6_interfaces.msg import CompileDiagnostic, GripperState, StudioState
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
        self._executor_lock = threading.Lock()
        self._executor = None
        self._wake_gc = None
        self._pending = {}
        self._req_seq = 0

    def _trigger_wake(self):
        gc = self._wake_gc
        if gc is not None:
            try:
                gc.trigger()
            except Exception:
                pass

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
        self._trigger_wake()

    def request_command(self, command_code, reason=""):
        self._req_queue.put(("command", {"code": int(command_code), "reason": str(reason)}))
        self._trigger_wake()

    def request_gripper(self, action_code):
        self._req_queue.put(("gripper", {"action": int(action_code)}))
        self._trigger_wake()

    def shutdown(self):
        self._stop_requested = True
        self._trigger_wake()
        with self._executor_lock:
            executor = self._executor
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass
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
            lambda msg: None if self._stop_requested else self.studio_state_updated.emit(msg),
            state_qos,
        )
        node.create_subscription(
            GripperState,
            "/gripper/state",
            lambda msg: None if self._stop_requested else self.gripper_state_updated.emit(msg),
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

        def on_wake():
            self._drain_requests(compile_cli, command_cli, gripper_cli)

        def on_housekeeping():
            if self._stop_requested:
                return
            self._sweep_pending_timeouts()
            self._emit_telemetry(shm_reader, pin_model, pin_data, eef_frame_id)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        wake_gc = node.create_guard_condition(on_wake)
        housekeeping_timer = node.create_timer(0.05, on_housekeeping)

        should_spin = False
        with self._executor_lock:
            if not self._stop_requested:
                self._wake_gc = wake_gc
                self._executor = executor
                should_spin = True

        try:
            if should_spin and rclpy.ok():
                on_wake()
                executor.spin()
        except Exception as exc:
            if not self._stop_requested:
                self.ros_log.emit(f"ROS executor error: {exc}")
        finally:
            self._wake_gc = None
            with self._executor_lock:
                self._executor = None
            try:
                node.destroy_timer(housekeeping_timer)
            except Exception:
                pass
            try:
                node.destroy_guard_condition(wake_gc)
            except Exception:
                pass
            try:
                executor.remove_node(node)
            except Exception:
                pass
            try:
                executor.shutdown()
            except Exception:
                pass
            shm_reader.close()
            node.destroy_node()

    def _emit_telemetry(self, shm_reader, pin_model, pin_data, eef_frame_id):
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

    def _emit_compile_error(self, stage, code, message):
        resp = CompileProgram.Response()
        resp.success = False
        diag = CompileDiagnostic()
        diag.severity = CompileDiagnostic.SEVERITY_ERROR
        diag.stage = stage
        diag.line_number = 1
        diag.column_number = 1
        diag.statement_index = -1
        diag.code = code
        diag.message = message
        resp.diagnostics.append(diag)
        self.compile_finished.emit(resp)

    def _register_pending(self, kind, meta, fut, timeout_sec):
        self._req_seq += 1
        req_id = self._req_seq
        self._pending[req_id] = (kind, meta, fut, time.monotonic() + timeout_sec)
        fut.add_done_callback(lambda completed_fut, rid=req_id: self._on_future_done(rid, completed_fut))

    def _on_future_done(self, req_id, fut):
        entry = self._pending.pop(req_id, None)
        if entry is None or self._stop_requested:
            return
        kind, meta, _, _ = entry
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
            return

        if kind == "compile":
            self.compile_finished.emit(res)
        elif kind == "command":
            self.command_finished.emit(meta["code"], bool(res.accepted), str(res.message))
        else:
            self.gripper_finished.emit(meta["action"], bool(res.accepted), str(res.detail))

    def _sweep_pending_timeouts(self):
        now = time.monotonic()
        expired = [
            req_id for req_id, (_, _, _, deadline) in self._pending.items() if now > deadline
        ]
        for req_id in expired:
            entry = self._pending.pop(req_id, None)
            if entry is None:
                continue
            kind, meta, fut, _ = entry
            try:
                fut.cancel()
            except Exception:
                pass
            if kind == "compile":
                self._emit_compile_error(
                    CompileDiagnostic.STAGE_PLANNING,
                    "ERR_COMPILE_TIMEOUT",
                    "Compilation service call timed out",
                )
            elif kind == "command":
                self.command_finished.emit(meta["code"], False, "StudioCommand call timed out")
            else:
                self.gripper_finished.emit(meta["action"], False, "GripperCommand call timed out")

    def _drain_requests(self, compile_cli, command_cli, gripper_cli):
        while not self._stop_requested:
            try:
                kind, payload = self._req_queue.get_nowait()
            except queue.Empty:
                break

            if kind == "compile":
                if not compile_cli.service_is_ready():
                    self._emit_compile_error(
                        CompileDiagnostic.STAGE_SEMANTIC,
                        "ERR_SERVICE_OFFLINE",
                        "Service /studio/compile is unavailable (is lite6_studio_planner_node running?)",
                    )
                    continue
                req = CompileProgram.Request()
                req.source_code = payload["source_code"]
                req.use_current_robot_start = payload["use_current"]
                req.custom_start_joints = payload["custom_start"]
                fut = compile_cli.call_async(req)
                self._register_pending("compile", {}, fut, 20.0)

            elif kind == "command":
                code = payload["code"]
                if not command_cli.service_is_ready():
                    self.command_finished.emit(
                        code, False, "Service /studio/command is unavailable"
                    )
                    continue
                req = StudioCommand.Request()
                req.command = code
                req.reason = payload["reason"]
                fut = command_cli.call_async(req)
                self._register_pending("command", {"code": code}, fut, 8.0)

            elif kind == "gripper":
                action = payload["action"]
                if not gripper_cli.service_is_ready():
                    self.gripper_finished.emit(
                        action, False, "Service /gripper/command is unavailable"
                    )
                    continue
                req = GripperCommand.Request()
                req.command = action
                fut = gripper_cli.call_async(req)
                self._register_pending("gripper", {"action": action}, fut, 5.0)
