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
from lite6_interfaces.srv import (
    CompileProgram, ControllerCommand, GripperCommand, StudioCommand,
)


class StudioRosWorker(QThread):
    """Runs an isolated rclpy node loop and dispatches telemetry/service results via Qt signals."""

    studio_state_updated = pyqtSignal(object)
    gripper_state_updated = pyqtSignal(object)
    telemetry_updated = pyqtSignal(object, object, object)  # q_list, dq_list, tcp6_or_none
    compile_finished = pyqtSignal(object)
    command_finished = pyqtSignal(int, bool, str)
    gripper_finished = pyqtSignal(int, bool, str)
    supervisor_offline = pyqtSignal(str)
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
        self._compile_req_seq = 0
        self._revision = -1
        self._last_studio_rx_time = 0.0
        self._boot_time = time.monotonic()
        self._direct_estop_sent = False
        self._last_studio_state_code = None

    def _trigger_wake(self):
        gc = self._wake_gc
        if gc is not None:
            try:
                gc.trigger()
            except Exception:
                pass

    def request_compile(self, source_code, use_current_robot_start=True, custom_start=None, auto_load=True):
        self._compile_req_seq += 1
        self._req_queue.put(
            (
                "compile",
                {
                    "source_code": str(source_code),
                    "use_current": bool(use_current_robot_start),
                    "custom_start": list(custom_start) if custom_start is not None else [0.0] * 6,
                    "request_id": int(self._compile_req_seq),
                    "auto_load": bool(auto_load),
                },
            )
        )
        self._trigger_wake()

    def request_command(self, command_code, reason="", expected_revision=0):
        self._req_queue.put(
            (
                "command",
                {
                    "code": int(command_code),
                    "reason": str(reason),
                    "expected_revision": int(expected_revision),
                },
            )
        )
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
            self._on_studio_state,
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
        ctc_cmd_cli = node.create_client(ControllerCommand, "/ctc_controller/command")

        def on_wake():
            self._drain_requests(compile_cli, command_cli, gripper_cli, ctc_cmd_cli)

        def on_housekeeping():
            if self._stop_requested:
                return
            self._sweep_pending_timeouts(ctc_cmd_cli)
            self._check_supervisor_watchdog(ctc_cmd_cli)
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

    def _on_studio_state(self, msg):
        if self._stop_requested:
            return
        now = time.monotonic()
        if self._last_studio_rx_time and (now - self._last_studio_rx_time > 0.8):
            # Planner restarted or went stale: reset monotonic revision filter.
            self._revision = -1
        if msg.revision < self._revision:
            return
        self._revision = int(msg.revision)
        self._last_studio_rx_time = now
        self._last_studio_state_code = int(msg.state)
        if msg.controller_mode == 1 and msg.controller_fresh:
            self._direct_estop_sent = False
        self.studio_state_updated.emit(msg)

    def _send_direct_controller_estop(self, ctc_cmd_cli, reason, force=False):
        """Latch E-STOP directly on the CTC controller.

        force=True bypasses the one-shot fallback latch; used for operator E-STOP
        requests, which are always sent to the controller in parallel with the
        StudioCommand so a busy or hung Studio planner can never delay them.
        """
        if self._direct_estop_sent and not force:
            return
        if not ctc_cmd_cli.service_is_ready():
            return
        self._direct_estop_sent = True
        req = ControllerCommand.Request()
        req.command = ControllerCommand.Request.CMD_ESTOP
        req.expected_epoch = 0
        req.reason = str(reason)
        ctc_cmd_cli.call_async(req)
        self.ros_log.emit(f"CTC DIRECT ESTOP FALLBACK: {reason}")

    def _check_supervisor_watchdog(self, ctc_cmd_cli):
        now = time.monotonic()
        if not self._last_studio_rx_time:
            return
        age = now - self._last_studio_rx_time
        if age > 1.2:
            self._revision = -1
            if not self._direct_estop_sent:
                if self._last_studio_state_code in (
                    StudioState.STATE_RUNNING,
                    StudioState.STATE_STEPPING,
                    StudioState.STATE_STOPPING,
                    StudioState.STATE_RECOVERING,
                ):
                    self._send_direct_controller_estop(
                        ctc_cmd_cli, "Studio supervisor heartbeat lost during motion"
                    )
                self.supervisor_offline.emit(
                    f"Studio supervisor heartbeat lost ({age:.1f}s)"
                )

    def _emit_telemetry(self, shm_reader, pin_model, pin_data, eef_frame_id):
        pod, _ = shm_reader.read_joint_state()
        if pod is None or pod.steady_stamp_ns <= 0:
            return
        if steady_now_ns() - pod.steady_stamp_ns > 100_000_000:
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
                if meta.get("code") == StudioCommand.Request.CMD_ESTOP and meta.get("ctc_cli") is not None:
                    self._send_direct_controller_estop(meta["ctc_cli"], str(exc))
                self.command_finished.emit(meta["code"], False, str(exc))
            else:
                self.gripper_finished.emit(meta["action"], False, str(exc))
            return

        if kind == "compile":
            self.compile_finished.emit(res)
        elif kind == "command":
            if (not res.accepted and
                    meta.get("code") == StudioCommand.Request.CMD_ESTOP and
                    meta.get("ctc_cli") is not None):
                self._send_direct_controller_estop(meta["ctc_cli"], str(res.message))
            self.command_finished.emit(meta["code"], bool(res.accepted), str(res.message))
        else:
            self.gripper_finished.emit(meta["action"], bool(res.accepted), str(res.detail))

    def _sweep_pending_timeouts(self, ctc_cmd_cli):
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
                if meta.get("code") == StudioCommand.Request.CMD_ESTOP:
                    self._send_direct_controller_estop(
                        ctc_cmd_cli, "StudioCommand CMD_ESTOP timed out"
                    )
                self.command_finished.emit(meta["code"], False, "StudioCommand call timed out")
            else:
                self.gripper_finished.emit(meta["action"], False, "GripperCommand call timed out")

    def _drain_requests(self, compile_cli, command_cli, gripper_cli, ctc_cmd_cli):
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
                req.request_id = payload["request_id"]
                req.auto_load = payload["auto_load"]
                fut = compile_cli.call_async(req)
                self._register_pending("compile", {}, fut, 20.0)

            elif kind == "command":
                code = payload["code"]
                if code == StudioCommand.Request.CMD_ESTOP:
                    # CTC E-STOP is idempotent; the Studio supervisor follows by observation.
                    self._send_direct_controller_estop(
                        ctc_cmd_cli, payload["reason"] or "Operator E-STOP", force=True
                    )
                if not command_cli.service_is_ready():
                    if code == StudioCommand.Request.CMD_ESTOP:
                        self._send_direct_controller_estop(
                            ctc_cmd_cli, payload["reason"] or "Studio service unavailable"
                        )
                    self.command_finished.emit(
                        code, False, "Service /studio/command is unavailable"
                    )
                    continue
                req = StudioCommand.Request()
                req.command = code
                req.expected_revision = (
                    0 if code == StudioCommand.Request.CMD_ESTOP
                    else int(payload.get("expected_revision", 0))
                )
                req.reason = payload["reason"]
                fut = command_cli.call_async(req)
                self._register_pending(
                    "command", {"code": code, "ctc_cli": ctc_cmd_cli}, fut, 8.0
                )

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
