"""ROS 2 transport adapter for the Robot GUI.

The planner's Supervisor is the *only* owner of the global discrete state.
This module never transitions the global machine: it translates state snapshots
into the existing HMI labels and sends guarded commands to the Supervisor.

GUI widgets, their layouts and the SysIdEngine numerical methods live elsewhere.
"""

import math
import queue
import threading
import time
import uuid

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID
from lite6_interfaces.action import IndustrialMotion
from lite6_interfaces.msg import SupervisorState, MotionRejection, GripperState
from lite6_interfaces.srv import (
    ControllerCommand, SupervisorCommand, SubmitSysIdTrajectory, GripperCommand,
)
from lite6_shm_client import StateShmReader, SysIdShmReader, steady_now_ns
from PyQt5.QtCore import QThread, pyqtSignal

from lite6_hmi.sysid_engine import SysIdEngine
from lite6_hmi.teaching_recording import TeachingRecording


class Lite6StateMachine:
    """Legacy *view labels*. These are NOT writable states or an FSM."""
    INIT = "INITIALIZING..."
    IDLE = "IDLE (Ready)"
    PLANNING = "PLANNING..."
    EXECUTING = "EXECUTING..."
    ERROR = "HARDWARE E-STOP!"
    RECOVERING = "RECOVERING..."
    SOFT_ERROR = "SOFTWARE FAULT"
    SYSID_PRE_ALIGN = "FRICTION ID: PRE-ALIGNING..."
    SYSID_SEND_CMD = "FRICTION ID: CONFIGURING HARDWARE..."
    SYSID_RUNNING = "FRICTION ID: EXCITING TRAJECTORY..."
    SYSID_CALC = "FRICTION ID: ESTIMATING FRICTION..."
    SYSID_POST_ALIGN = "FRICTION ID: ALIGNING CTC TARGETS..."
    SYSID_RESTORE = "FRICTION ID: RESTORING CLOSED-LOOP..."


def _view_state(msg):
    """Pure projection of authoritative SupervisorState into existing GUI labels."""
    S = SupervisorState
    if msg.mode == S.MODE_INITIALIZING:
        return Lite6StateMachine.INIT
    if msg.mode in (S.MODE_ESTOP, S.MODE_FAULT):
        return Lite6StateMachine.ERROR
    if msg.mode == S.MODE_SOFT_FAULT:
        return Lite6StateMachine.SOFT_ERROR
    if msg.mode == S.MODE_RECOVERING:
        return Lite6StateMachine.RECOVERING
    compliance_labels = {
        S.MODE_JOINT_IMPEDANCE: "JOINT IMPEDANCE",
        S.MODE_CARTESIAN_IMPEDANCE: "CARTESIAN IMPEDANCE",
        S.MODE_TEACH: "DRAG TEACHING",
        S.MODE_COMPLIANCE_SWITCHING: "COMPLIANCE: SWITCHING / STOPPING...",
    }
    if msg.mode in compliance_labels:
        return compliance_labels[msg.mode]
    if msg.mode == S.MODE_SYSID:
        if msg.waiting_for_settle:
            return "FRICTION ID: WAITING FOR PHYSICAL SETTLING..."
        p = msg.sysid_phase
        labels = {
            S.SYSID_PRE_ALIGN: Lite6StateMachine.SYSID_PRE_ALIGN,
            S.SYSID_WAIT_CONTROLLER_MODE: Lite6StateMachine.SYSID_SEND_CMD,
            S.SYSID_READY_FOR_EXCITATION: "FRICTION ID: READY FOR EXCITATION...",
            S.SYSID_VALIDATING: "FRICTION ID: VALIDATING TRAJECTORY...",
            S.SYSID_EXCITING: Lite6StateMachine.SYSID_RUNNING,
            S.SYSID_CALCULATING: Lite6StateMachine.SYSID_CALC,
            S.SYSID_POST_ALIGN: Lite6StateMachine.SYSID_POST_ALIGN,
            S.SYSID_WAIT_NORMAL_MODE: Lite6StateMachine.SYSID_RESTORE,
            S.SYSID_RETURN_ALIGN: "FRICTION ID: RETURNING TO ALIGNMENT POSE...",
            S.SYSID_COMPLETING: "FRICTION ID: CONFIRMING COMPLETION...",
        }
        return labels.get(p, "FRICTION ID: WAITING...")
    if msg.motion_phase == S.MOTION_PLANNING:
        return Lite6StateMachine.PLANNING
    if msg.motion_phase == S.MOTION_WAITING_CTC:
        return "WAITING FOR CTC..."
    if msg.motion_phase == S.MOTION_EXECUTING:
        return Lite6StateMachine.EXECUTING
    if msg.motion_phase == S.MOTION_CANCELING:
        return "CANCELING MOTION..."
    return Lite6StateMachine.IDLE


class ROS2Worker(QThread):
    telemetry_signal = pyqtSignal(dict)
    state_signal = pyqtSignal(str, str)
    sysid_yaml_signal = pyqtSignal(str)
    sysid_preparation_signal = pyqtSignal()
    command_error_signal = pyqtSignal(str)
    estop_fault_signal = pyqtSignal(str)
    recovery_complete_signal = pyqtSignal()
    gripper_signal = pyqtSignal(object)
    motion_feedback_signal = pyqtSignal(int, float, float)

    def __init__(self):
        super().__init__()
        self.node = Node("lite6_industrial_hmi")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.state_lock = threading.RLock()
        self.state = Lite6StateMachine.INIT  # UI projection, not writable state
        self.error_msg = ""
        self.supervisor_state = None
        self._revision = -1
        self._last_view = (None, None)
        self.boot_time = time.monotonic()
        self.last_supervisor_rx_time = 0.0
        self.last_state_rx_time = 0.0  # joint feedback receipt time, monotonic
        self.last_ui_update_time = 0.0
        self._stopping = False
        self._bg_threads_lock = threading.Lock()
        self._bg_threads = set()

        self.joint_names = [f"joint{i}" for i in range(1, 7)]
        self.teaching = TeachingRecording()
        self.q_curr = [0.0] * 6
        self.dq_curr = [0.0] * 6
        self.tau_curr = [0.0] * 6
        self.pose_curr = [0.0] * 6

        self.move_client = ActionClient(self.node, IndustrialMotion, "industrial_motion")
        self.command_client = self.node.create_client(
            SupervisorCommand, "/supervisor/command")
        self.sysid_client = self.node.create_client(
            SubmitSysIdTrajectory, "/supervisor/submit_sysid_trajectory")
        # Emergency fallback only. Normal and SYS_ID transitions are *never*
        # commanded directly by the GUI.
        self.ctc_cmd_client = self.node.create_client(
            ControllerCommand, "/ctc_controller/command")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.supervisor_sub = self.node.create_subscription(
            SupervisorState, "/supervisor/state", self._supervisor_cb, qos)
        self._state_shm = StateShmReader()
        self._sysid_shm = SysIdShmReader()
        self._last_joint_seq = 0
        self._last_joint_steady_ns = 0
        self._sysid_shm_enabled = False
        self.shm_timer = self.node.create_timer(0.01, self._poll_shm)
        self._ros_tasks = queue.Queue()
        self._executor_thread_id = None
        self._ros_wake = self.node.create_guard_condition(self._drain_ros_tasks)
        self.gripper_state = None
        self.last_gripper_rx_time = 0.0
        self.gripper_client = self.node.create_client(GripperCommand, "/gripper/command")
        self.gripper_sub = self.node.create_subscription(
            GripperState, "/gripper/state", self._gripper_cb, 1)

        self.current_goal_handle = None
        self._motion_pending = False
        self._goal_epoch = 0
        self._result_reported_epoch = -1
        # Correlate admission diagnostics with the actual Action response.
        # The topic and response can arrive in either order.
        self._motion_requests = {}
        self.rejection_sub = self.node.create_subscription(
            MotionRejection, "/supervisor/motion_rejections", self._motion_rejection_cb, 32)
        self.rejection_timer = self.node.create_timer(0.1, self._flush_motion_rejections)
        self._begin_pending = False
        self.sysid_preparation_status = ""
        self._prepare_epoch = 0
        self._prepared_trajectory = None
        self._sysid_session = 0
        self._recovery_session = 0
        self._phase_actions = set()
        self._calculation_active = False
        self._estop_requested_at = 0.0
        self._direct_estop_sent = False

        self.sysid_record_data = []
        self.sysid_start_ros_time = None
        self.sysid_q0_locked = [0.0] * 6

        try:
            import os
            path = os.path.join(get_package_share_directory("lite6_description"),
                                "urdf", "lite6_arm.urdf")
            self.model = pin.buildModelFromUrdf(path)
            self.data = self.model.createData()
            self.eef_frame_id = self.model.getFrameId("link_eef")
            self.node.get_logger().info("Pinocchio GUI FK model initialized")
        except Exception as exc:
            self.model = None
            self.node.get_logger().error(f"GUI FK initialization failed: {exc}")
        try:
            self.sysid_calc = SysIdEngine()
        except Exception as exc:
            self.sysid_calc = None
            self.node.get_logger().error(f"SysIdEngine initialization failed: {exc}")

    def _gripper_cb(self, msg):
        with self.state_lock:
            self.gripper_state = msg
            self.last_gripper_rx_time = time.monotonic()
        self.gripper_signal.emit(msg)

    def gripper_ready(self):
        with self.state_lock:
            g = self.gripper_state
            return bool(g is not None and g.ready and g.active
                        and time.monotonic() - self.last_gripper_rx_time < 0.25)

    def _drain_ros_tasks(self):
        while True:
            try:
                fn = self._ros_tasks.get_nowait()
            except queue.Empty:
                break
            try:
                fn()
            except Exception as exc:
                if not self._stopping:
                    self.node.get_logger().warn(f"Queued ROS task error: {exc}")

    def _run_on_executor(self, fn):
        if self._stopping:
            return False
        if self._executor_thread_id is not None and threading.get_ident() == self._executor_thread_id:
            fn()
            return True
        self._ros_tasks.put(fn)
        try:
            self._ros_wake.trigger()
        except Exception:
            pass
        return True

    def _spawn_bg_thread(self, target, args=()):
        if self._stopping:
            return
        def _runner():
            try:
                target(*args)
            finally:
                with self._bg_threads_lock:
                    self._bg_threads.discard(threading.current_thread())
        thread = threading.Thread(target=_runner, daemon=True)
        with self._bg_threads_lock:
            if self._stopping:
                return
            self._bg_threads.add(thread)
        thread.start()

    def command_gripper(self, command):
        err = None
        with self.state_lock:
            if not self.gripper_ready():
                err = "Gripper hardware is inactive or feedback is stale"
        if err is not None:
            self._report_error(err)
            return
        if not self._run_on_executor(lambda cmd=command: self._send_gripper_on_executor(cmd)):
            self._report_error("Gripper request canceled while stopping")

    def _send_gripper_on_executor(self, command):
        err = None
        with self.state_lock:
            if not self.gripper_ready():
                err = "Gripper hardware is inactive or feedback is stale"
        if err is not None:
            self._report_error(err)
            return
        if not self.gripper_client.service_is_ready():
            self._report_error("Gripper service unavailable")
            return
        request = GripperCommand.Request()
        request.command = command
        try:
            self.gripper_client.call_async(request).add_done_callback(self._gripper_response)
        except Exception as exc:
            self._report_error(f"Gripper request failed: {exc}")

    def _gripper_response(self, future):
        try:
            response = future.result()
            if not response.accepted:
                self._report_error(f"Gripper rejected: {response.detail}")
        except Exception as exc:
            self._report_error(f"Gripper request failed: {exc}")

    def run(self):
        self._executor_thread_id = threading.get_ident()
        self._drain_ros_tasks()
        try:
            self.executor.spin()
        finally:
            self._drain_ros_tasks()
            self.executor.remove_node(self.node)

    def stop(self):
        self._stopping = True
        try:
            self._ros_wake.trigger()
        except Exception:
            pass
        self.executor.shutdown(timeout_sec=2.0)
        self.wait(2500)
        with self._bg_threads_lock:
            bg_threads = list(self._bg_threads)
        deadline = time.monotonic() + 2.0
        for thread in bg_threads:
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                thread.join(timeout=remaining)
        self._sysid_shm.close()
        self._state_shm.close()
        try:
            self.node.destroy_guard_condition(self._ros_wake)
        except Exception:
            pass
        self.node.destroy_node()

    def _report_error(self, message):
        if not self._stopping:
            self.node.get_logger().warn(message)
            self.command_error_signal.emit(message)

    def _motion_rejection_cb(self, msg):
        with self.state_lock:
            request = self._motion_requests.get(msg.goal_id)
            if request is None:
                return  # Another client, old request, or an already reported rejection.
            request['detail'] = (
                f"Motion rejected: {self.get_planner_error_str(msg.error_code)} "
                f"({msg.error_code})\n{msg.error_string}")
        self._flush_motion_rejections()

    def _flush_motion_rejections(self):
        messages = []
        now = time.monotonic()
        with self.state_lock:
            for key, request in list(self._motion_requests.items()):
                if request['rejected_at'] is not None:
                    if request['detail'] is not None or now - request['rejected_at'] >= 1.0:
                        messages.append(request['detail'] or
                                        "IndustrialMotion goal rejected by Supervisor\n"
                                        "Detailed admission diagnostic was not received.")
                        del self._motion_requests[key]
                elif now - request['created'] > 30.0:
                    # Bounds diagnostic storage only; never changes motion authorization.
                    del self._motion_requests[key]
        for message in messages:
            self._report_error(message)

    def _send_command(self, event, session=0, reason="", done=None, compliance=None,
                      expected_revision=0, payload_mass=0.0, payload_com_tcp=None):
        if self._executor_thread_id is not None and threading.get_ident() == self._executor_thread_id:
            return self._send_command_on_executor(
                event, session, reason, done, compliance, expected_revision,
                payload_mass, payload_com_tcp)
        return self._run_on_executor(
            lambda: self._send_command_on_executor(
                event, session, reason, done, compliance, expected_revision,
                payload_mass, payload_com_tcp))

    def _send_command_on_executor(self, event, session=0, reason="", done=None,
                                  compliance=None, expected_revision=0,
                                  payload_mass=0.0, payload_com_tcp=None):
        if not self.command_client.service_is_ready():
            self._report_error("Supervisor command service is unavailable")
            if done is not None:
                done(None)
            return False
        req = SupervisorCommand.Request()
        req.command = event
        req.expected_revision = int(expected_revision)
        req.session_id = int(session)
        req.reason = str(reason)
        if compliance is not None:
            req.compliance = compliance
        req.payload_mass = float(payload_mass)
        if payload_com_tcp is not None:
            req.payload_com_tcp = [float(payload_com_tcp[0]),
                                   float(payload_com_tcp[1]),
                                   float(payload_com_tcp[2])]
        try:
            future = self.command_client.call_async(req)
            future.add_done_callback(lambda f: self._command_response(f, done))
            return True
        except Exception as exc:
            self._report_error(f"Supervisor command request failed: {exc}")
            if done is not None:
                done(None)
            return False

    def set_payload(self, mass_kg, com_tcp_m, done=None):
        expected_rev = 0
        with self.state_lock:
            if self.supervisor_state is not None:
                expected_rev = int(self.supervisor_state.revision)
        return self._send_command(
            SupervisorCommand.Request.CMD_SET_PAYLOAD,
            reason=f"HMI set payload {mass_kg:.3f} kg",
            done=done,
            expected_revision=expected_rev,
            payload_mass=float(mass_kg),
            payload_com_tcp=list(com_tcp_m),
        )

    def _command_response(self, future, done):
        try:
            resp = future.result()
            if not resp.accepted:
                self._report_error(f"Supervisor denied event: {resp.message}")
            if done is not None:
                done(resp)
        except Exception as exc:
            self._report_error(f"Supervisor command response failed: {exc}")
            if done is not None:
                done(None)

    def _supervisor_cb(self, msg):
        now = time.monotonic()
        with self.state_lock:
            # Reset monotonic revision filter if supervisor went stale (e.g. planner restart).
            if self.last_supervisor_rx_time and (now - self.last_supervisor_rx_time > 0.8):
                self._revision = -1
            # A stale latched / out-of-order message cannot roll back the view.
            if msg.revision < self._revision:
                return
            previous = self.supervisor_state
            if (self._sysid_shm_enabled and previous is not None and
                    previous.mode == SupervisorState.MODE_SYSID and
                    previous.sysid_phase == SupervisorState.SYSID_EXCITING and
                    (msg.mode != SupervisorState.MODE_SYSID or
                     msg.sysid_phase != SupervisorState.SYSID_EXCITING)):
                # Drain trailing samples pushed before leaving SYSID_EXCITING.
                self._drain_sysid_shm_unlocked(update_heartbeat=False)
            self.last_supervisor_rx_time = now
            if msg.revision == self._revision and previous is not None:
                # Repeated heartbeats may update freshness flags but do not
                # represent new transitions. Store their latest projection.
                pass
            self._revision = msg.revision
            self.supervisor_state = msg
            if msg.mode != SupervisorState.MODE_TEACH:
                self.teaching.stop()
            # Enable SysID ring capture during preparation/alignment so zero
            # excitation samples are dropped on entry.
            if msg.mode == SupervisorState.MODE_SYSID and not self._sysid_shm_enabled:
                self._sysid_shm.discard_all()
                self._sysid_shm.set_active(True)
                self._sysid_shm_enabled = True
            elif msg.mode != SupervisorState.MODE_SYSID and self._sysid_shm_enabled:
                self._sysid_shm.set_active(False)
                self._sysid_shm_enabled = False
            if msg.mode == SupervisorState.MODE_SYSID and msg.session_id == self._sysid_session:
                if self._begin_pending:
                    self._set_sysid_preparation_status("")
                if msg.excitation_start.sec or msg.excitation_start.nanosec:
                    self.sysid_start_ros_time = (msg.excitation_start.sec +
                                                 msg.excitation_start.nanosec * 1e-9)
            self.state = _view_state(msg)
            self.error_msg = msg.detail if msg.mode in (
                SupervisorState.MODE_ESTOP, SupervisorState.MODE_FAULT,
                SupervisorState.MODE_SOFT_FAULT) else ""
            display = (self.state, self.error_msg)
            changed = display != self._last_view
            self._last_view = display
            entered_estop = (
                msg.mode in (SupervisorState.MODE_ESTOP, SupervisorState.MODE_FAULT)
                and (previous is None or previous.mode not in (
                    SupervisorState.MODE_ESTOP, SupervisorState.MODE_FAULT))
            )
            # Suppress the fault popup for intentional manual E-STOP presses and
            # for trajectory failures that already report through _goal_result.
            trajectory_reports_via_action = (
                msg.estop_reason == SupervisorState.ESTOP_TRAJECTORY
                and previous is not None
                and previous.motion_phase != SupervisorState.MOTION_IDLE
            )
            estop_popup = None
            if (entered_estop
                    and msg.estop_reason != SupervisorState.ESTOP_MANUAL
                    and not trajectory_reports_via_action):
                estop_popup = msg.detail or "Controller entered emergency stop mode."
            if msg.mode == SupervisorState.MODE_ESTOP:
                self._estop_requested_at = 0.0
            # After an observed, completed recovery, another independent
            # emergency fallback must be possible in the next operating cycle.
            if msg.mode == SupervisorState.MODE_NORMAL and msg.controller_fresh:
                self._direct_estop_sent = False
            if msg.mode in (SupervisorState.MODE_ESTOP,
                            SupervisorState.MODE_FAULT):
                if self._begin_pending:
                    self._prepare_epoch += 1
                    self._set_sysid_preparation_status("")
                self._sysid_session = 0
                self._recovery_session = 0
                self._phase_actions.clear()
            if (previous is not None
                    and previous.mode == SupervisorState.MODE_RECOVERING
                    and msg.mode == SupervisorState.MODE_NORMAL
                    and self._recovery_session != 0):
                self._recovery_session = 0
                self.recovery_complete_signal.emit()
            if (previous is not None
                    and previous.mode == SupervisorState.MODE_SYSID
                    and msg.mode == SupervisorState.MODE_NORMAL):
                self._sysid_session = 0
                self._phase_actions.clear()
                self.node.get_logger().info(msg.detail)
        if changed:
            self.state_signal.emit(display[0], display[1])
        if estop_popup is not None:
            self.estop_fault_signal.emit(estop_popup)
        self._drive_supervisor_phase(msg)

    def _drive_supervisor_phase(self, msg):
        """Entry actions only; the Supervisor decides every next state."""
        with self.state_lock:
            sid = self._sysid_session
            if (not sid or msg.session_id != sid or
                    msg.mode != SupervisorState.MODE_SYSID or msg.waiting_for_settle):
                return
            phase = msg.sysid_phase
            eligible = {
                SupervisorState.SYSID_PRE_ALIGN,
                SupervisorState.SYSID_READY_FOR_EXCITATION,
                SupervisorState.SYSID_CALCULATING,
                SupervisorState.SYSID_POST_ALIGN,
                SupervisorState.SYSID_RETURN_ALIGN,
            }
            key = (sid, phase)
            if phase not in eligible or key in self._phase_actions:
                return
            self._phase_actions.add(key)
            prep = list(self.sysid_q0_locked)
            q_now = list(self.q_curr)
        if phase == SupervisorState.SYSID_PRE_ALIGN:
            if not self.execute_joint_target(
                    prep, 1.0, 1.0,
                    context=IndustrialMotion.Goal.CONTEXT_SYSID, session_id=sid):
                self._abort_sysid(sid, "Could not dispatch pre-alignment")
        elif phase == SupervisorState.SYSID_READY_FOR_EXCITATION:
            self._spawn_bg_thread(self._submit_sysid_trajectory, (sid,))
        elif phase == SupervisorState.SYSID_CALCULATING:
            self.run_sysid_calculation(sid)
        elif phase == SupervisorState.SYSID_POST_ALIGN:
            if not self.execute_joint_target(
                    q_now, 0.15, 0.15,
                    context=IndustrialMotion.Goal.CONTEXT_SYSID, session_id=sid):
                self._abort_sysid(sid, "Could not dispatch post-alignment")
        elif phase == SupervisorState.SYSID_RETURN_ALIGN:
            if not self.execute_joint_target(
                    prep, 0.15, 0.15,
                    context=IndustrialMotion.Goal.CONTEXT_SYSID, session_id=sid):
                self._abort_sysid(sid, "Could not dispatch return-alignment")

    @staticmethod
    def _ns_to_sec(ns: int) -> float:
        sec, nsec = divmod(int(ns), 1_000_000_000)
        return sec + nsec * 1e-9

    def _poll_shm(self):
        self._poll_joint_shm()
        if self._sysid_shm_enabled:
            with self.state_lock:
                self._drain_sysid_shm_unlocked(update_heartbeat=True)

    def _drain_sysid_shm_unlocked(self, update_heartbeat: bool = True):
        if update_heartbeat:
            self._sysid_shm.set_active(True)
        s = self.supervisor_state
        if (s is not None and
                s.mode == SupervisorState.MODE_SYSID and
                s.sysid_phase == SupervisorState.SYSID_VALIDATING):
            return
        samples = self._sysid_shm.pop_all()
        if not samples:
            return
        if (self.sysid_calc is None or s is None or
                s.mode != SupervisorState.MODE_SYSID or
                s.sysid_phase != SupervisorState.SYSID_EXCITING or
                s.session_id != self._sysid_session):
            return
        for pod in samples:
            self.sysid_record_data.append({
                't': self._ns_to_sec(pod.stamp_ns),
                'read_time': self._ns_to_sec(pod.read_stamp_ns),
                'simulation_time': float(pod.simulation_time),
                'step': int(pod.step),
                'command_sequence': int(pod.command_sequence),
                'control_period': float(pod.control_period),
                'physics_lag': float(pod.physics_lag),
                'command_age': float(pod.command_age),
                'q': np.array(pod.position, dtype=float),
                'dq': np.array(pod.velocity, dtype=float),
                'tau_cmd': np.array(pod.effort, dtype=float),
                'gripper_q': np.array(pod.gripper_position, dtype=float),
                'gripper_dq': np.array(pod.gripper_velocity, dtype=float),
                'gripper_ddq': np.array(pod.gripper_acceleration, dtype=float),
            })

    def _poll_joint_shm(self):
        pod, seq = self._state_shm.read_joint_state()
        if pod is None or seq == self._last_joint_seq:
            return
        now_steady_ns = steady_now_ns()
        if pod.steady_stamp_ns <= 0 or now_steady_ns - pod.steady_stamp_ns >= 100_000_000:
            return
        if pod.steady_stamp_ns <= self._last_joint_steady_ns:
            return
        self._last_joint_seq = seq
        self._last_joint_steady_ns = pod.steady_stamp_ns
        q_vals = [float(pod.position[i]) for i in range(6)]
        dq_vals = [float(pod.velocity[i]) for i in range(6)]
        tau_vals = [float(pod.effort[i]) for i in range(6)]
        if not all(math.isfinite(x) for x in q_vals + dq_vals):
            return
        monotonic_now = time.monotonic()
        with self.state_lock:
            self.q_curr = q_vals
            self.dq_curr = dq_vals
            self.tau_curr = tau_vals
            self.last_state_rx_time = monotonic_now
            self.teaching.observe(self._ns_to_sec(pod.ros_stamp_ns), self.q_curr)
            if monotonic_now - self.last_ui_update_time < 0.033:
                return
            if self.model is not None:
                try:
                    q = np.asarray(self.q_curr, dtype=float)
                    pin.forwardKinematics(self.model, self.data, q)
                    pin.updateFramePlacements(self.model, self.data)
                    frame = self.data.oMf[self.eef_frame_id]
                    rpy = pin.rpy.matrixToRpy(frame.rotation)
                    self.pose_curr = [*frame.translation, *rpy]
                except Exception as exc:
                    self.node.get_logger().warn(f"GUI forward kinematics: {exc}")
            packet = {
                "q": list(self.q_curr),
                "dq": list(self.dq_curr),
                "tau": list(self.tau_curr),
                "pose": list(self.pose_curr),
                "timestamp": self.last_state_rx_time,
            }
            self.last_ui_update_time = monotonic_now
        self.telemetry_signal.emit(packet)

    @staticmethod
    def _array_to_pose_msg(pose_arr):
        p = Pose()
        p.position.x, p.position.y, p.position.z = map(float, pose_arr[:3])
        matrix = pin.rpy.rpyToMatrix(np.array(pose_arr[3:6], dtype=float))
        quat = pin.Quaternion(matrix)
        p.orientation.x, p.orientation.y = float(quat.x), float(quat.y)
        p.orientation.z, p.orientation.w = float(quat.z), float(quat.w)
        return p

    def _dispatch_motion(self, goal):
        if self._executor_thread_id is not None and threading.get_ident() == self._executor_thread_id:
            return self._dispatch_motion_on_executor(goal)
        return self._run_on_executor(lambda: self._dispatch_motion_on_executor(goal))

    def _validate_motion_unlocked(self, goal):
        snap = self.supervisor_state
        if self._motion_pending:
            return "A motion request is already awaiting an Action response"
        if goal.context == IndustrialMotion.Goal.CONTEXT_NORMAL:
            if self._begin_pending:
                return "Wait for friction identification preparation before requesting manual motion"
            if snap is None or not snap.motion_allowed:
                return "Supervisor does not currently permit manual motion"
            return None
        if snap is None or snap.session_id != goal.session_id:
            return "Session-scoped motion is not authorized"
        return None

    def _dispatch_motion_on_executor(self, goal):
        with self.state_lock:
            err = self._validate_motion_unlocked(goal)
        if err is not None:
            self._report_error(err)
            return False
        if not self.move_client.server_is_ready():
            self._report_error("IndustrialMotion Action server unavailable")
            return False
        epoch = 0
        goal_id = None
        key = None
        with self.state_lock:
            err = self._validate_motion_unlocked(goal)
            if err is None:
                self._motion_pending = True
                self._goal_epoch += 1
                epoch = self._goal_epoch
                goal_id = uuid.uuid4()
                key = goal_id.hex
                self._motion_requests[key] = {
                    'created': time.monotonic(), 'rejected_at': None, 'detail': None}
        if err is not None:
            self._report_error(err)
            return False
        try:
            fut = self.move_client.send_goal_async(
                goal,
                goal_uuid=UUID(uuid=list(goal_id.bytes)),
                feedback_callback=lambda fb_msg, ep=epoch: self._goal_feedback(fb_msg, ep),
            )
            fut.add_done_callback(lambda f: self._goal_response(
                f, epoch, int(goal.context), int(goal.session_id), key))
            return True
        except Exception as exc:
            with self.state_lock:
                self._motion_requests.pop(key, None)
                if epoch == self._goal_epoch:
                    self._motion_pending = False
            self._report_error(f"Motion submission failed: {exc}")
            return False

    def execute_joint_target(self, target_q, v_scale, a_scale,
                             callback=None, context=IndustrialMotion.Goal.CONTEXT_NORMAL,
                             session_id=0):
        # callback argument retained for source compatibility, not used for
        # sequence transitions. Only SupervisorState advances SysID/recovery.
        goal = IndustrialMotion.Goal()
        goal.command_type = IndustrialMotion.Goal.MOVEJ
        goal.joint_target = [float(q) for q in target_q]
        goal.velocity_scale = float(v_scale)
        goal.acceleration_scale = float(a_scale)
        goal.context = int(context)
        goal.session_id = int(session_id)
        return self._dispatch_motion(goal)

    def execute_pose_target(self, target_pose, v_scale, a_scale, command_type):
        goal = IndustrialMotion.Goal()
        goal.command_type = command_type
        goal.pose_target = self._array_to_pose_msg(target_pose)
        goal.velocity_scale = float(v_scale)
        goal.acceleration_scale = float(a_scale)
        goal.context = IndustrialMotion.Goal.CONTEXT_NORMAL
        goal.session_id = 0
        return self._dispatch_motion(goal)

    def execute_circular_target(self, target_pose, aux_pose, v_scale, a_scale):
        goal = IndustrialMotion.Goal()
        goal.command_type = IndustrialMotion.Goal.MOVEC
        goal.pose_target = self._array_to_pose_msg(target_pose)
        goal.pose_aux = self._array_to_pose_msg(aux_pose)
        goal.velocity_scale = float(v_scale)
        goal.acceleration_scale = float(a_scale)
        goal.context = IndustrialMotion.Goal.CONTEXT_NORMAL
        goal.session_id = 0
        return self._dispatch_motion(goal)

    def _scoped_dispatch_failed(self, context, session, reason):
        if context == IndustrialMotion.Goal.CONTEXT_SYSID:
            self._abort_sysid(session, reason)

    def _goal_response(self, future, epoch, context, session, key):
        try:
            handle = future.result()
            cancel_handle = None
            stale = False
            rejected = False
            with self.state_lock:
                if epoch != self._goal_epoch:
                    stale = True
                    self._motion_requests.pop(key, None)
                    if handle is not None and handle.accepted:
                        cancel_handle = handle
                else:
                    self._motion_pending = False
                    if not handle.accepted:
                        request = self._motion_requests.setdefault(key, {
                            'created': time.monotonic(), 'detail': None})
                        request['rejected_at'] = time.monotonic()
                        rejected = True
                    else:
                        self._motion_requests.pop(key, None)
                        self.current_goal_handle = handle
            if cancel_handle is not None:
                cancel_handle.cancel_goal_async()
                return
            if stale:
                return
            if rejected:
                self._flush_motion_rejections()
                self._scoped_dispatch_failed(
                    context, session, "Supervisor rejected internal motion")
                return
            handle.get_result_async().add_done_callback(
                lambda f: self._goal_result(f, epoch, context, session))
        except Exception as exc:
            with self.state_lock:
                self._motion_requests.pop(key, None)
                if epoch == self._goal_epoch:
                    self._motion_pending = False
            self._report_error(f"Action goal response failed: {exc}")
            self._scoped_dispatch_failed(context, session, "Action response unavailable")

    def _goal_feedback(self, fb_msg, epoch):
        with self.state_lock:
            if epoch != self._goal_epoch:
                return
        fb = fb_msg.feedback
        self.motion_feedback_signal.emit(int(fb.current_state), float(fb.percent_complete), -1.0)

    def _goal_result(self, future, epoch, context, session):
        try:
            wrapped = future.result()
            with self.state_lock:
                if epoch != self._goal_epoch or epoch == self._result_reported_epoch:
                    return
                self._result_reported_epoch = epoch
                self.current_goal_handle = None
            result = wrapped.result
            self.motion_feedback_signal.emit(
                int(IndustrialMotion.Feedback.STATE_EXECUTING),
                100.0 if result.error_code == IndustrialMotion.Result.SUCCESS else 0.0,
                float(result.execution_time),
            )
            if result.error_code != IndustrialMotion.Result.SUCCESS:
                if result.error_code != IndustrialMotion.Result.ERR_CANCELLED:
                    # Only the Action result opens a notice. SupervisorState
                    # updates the banner, so heartbeat messages never repeat it.
                    self._report_error(
                        f"Motion failed: {self.get_planner_error_str(result.error_code)} "
                        f"({result.error_code})\n{result.error_string}")
                else:
                    self.node.get_logger().info(result.error_string)
                self._scoped_dispatch_failed(
                    context, session, f"Internal motion failed: {result.error_string}")
        except Exception as exc:
            self._report_error(f"Action result unavailable: {exc}")
            self._scoped_dispatch_failed(context, session, "Action result unavailable")

    def trigger_estop(self, reason, force_fallback=False):
        """Latch E-STOP directly on the CTC and request supervisor ESTOP in parallel.

        The direct /ctc_controller/command request is sent first, so a busy or hung
        supervisor (whose command service may wait on another CTC handshake) can
        never delay the RT latch. The supervisor request keeps SupervisorState and
        motion bookkeeping consistent.
        """
        with self.state_lock:
            self._estop_requested_at = time.monotonic()
        self._run_on_executor(lambda: self._trigger_estop_on_executor(reason, force_fallback))

    def _send_direct_ctc_estop_on_executor(self, reason):
        """Unconditional direct CTC E-STOP (bypasses the one-shot fallback latch)."""
        if not self.ctc_cmd_client.service_is_ready():
            self.node.get_logger().error(
                f"CTC DIRECT ESTOP unavailable ({reason}): /ctc_controller/command not ready")
            return False
        with self.state_lock:
            self._direct_estop_sent = True
        req = ControllerCommand.Request()
        req.command = ControllerCommand.Request.CMD_ESTOP
        req.expected_epoch = 0
        req.reason = str(reason)
        self.ctc_cmd_client.call_async(req)
        self.node.get_logger().warn(f"CTC DIRECT ESTOP: {reason}")
        return True

    def _trigger_estop_on_executor(self, reason, force_fallback=False):
        self._send_direct_ctc_estop_on_executor(reason)
        with self.state_lock:
            h = self.current_goal_handle
        if h is not None:
            try:
                h.cancel_goal_async()  # per-goal; Supervisor also preempts it
            except Exception:
                pass
        if force_fallback or not self.command_client.service_is_ready():
            return
        self._send_command_on_executor(
            SupervisorCommand.Request.CMD_ESTOP, reason=reason, done=self._estop_reply)

    def _estop_reply(self, resp):
        if resp is None or not resp.accepted:
            self._emergency_fallback("Supervisor did not accept emergency stop")

    def _emergency_fallback(self, reason):
        self._run_on_executor(lambda: self._emergency_fallback_on_executor(reason))

    def _emergency_fallback_on_executor(self, reason):
        with self.state_lock:
            if self._direct_estop_sent:
                return
            if not self.ctc_cmd_client.service_is_ready():
                self.node.get_logger().error(
                    f"CTC DIRECT ESTOP FALLBACK unavailable ({reason}): /ctc_controller/command not ready")
                return
            self._direct_estop_sent = True
        req = ControllerCommand.Request()
        req.command = ControllerCommand.Request.CMD_ESTOP
        req.expected_epoch = 0
        req.reason = str(reason)
        self.ctc_cmd_client.call_async(req)
        self.node.get_logger().error(f"CTC DIRECT ESTOP FALLBACK: {reason}")

    def check_watchdog(self):
        """Qt 500 ms timer invokes this; no independent global FSM."""
        now = time.monotonic()
        with self.state_lock:
            s = self.supervisor_state
            age = now - self.last_supervisor_rx_time if self.last_supervisor_rx_time else float("inf")
            pending_stop = self._estop_requested_at
            already_fallback = self._direct_estop_sent
            if self.last_supervisor_rx_time and age > 1.2:
                # Reset monotonic filter when supervisor feed goes stale so planner restarts are accepted.
                self._revision = -1
        if age > 1.2 and now - self.boot_time > 2.0:
            if not already_fallback:
                self.trigger_estop("SUPERVISOR HEARTBEAT LOST", force_fallback=True)
            return True
        if pending_stop and now - pending_stop > 0.6:
            if s is None or s.mode != SupervisorState.MODE_ESTOP:
                self._emergency_fallback("ESTOP not confirmed by supervisor")
        return False

    def begin_recovery(self):
        not_ready = False
        is_fault = False
        with self.state_lock:
            s = self.supervisor_state
            if s is not None and s.mode == SupervisorState.MODE_FAULT and not s.estop_latched:
                is_fault = True
            elif s is None or s.mode != SupervisorState.MODE_ESTOP:
                not_ready = True
        if is_fault:
            self.acknowledge_soft_fault()
            return
        if not_ready:
            self._report_error("Recovery requires Supervisor ESTOP mode")
            return
        self._send_command(SupervisorCommand.Request.CMD_BEGIN_RECOVERY,
                           done=self._recovery_reply)

    def acknowledge_soft_fault(self):
        self._send_command(SupervisorCommand.Request.CMD_ACK_SOFT_FAULT)

    def _recovery_reply(self, resp):
        if resp is None or not resp.accepted:
            return
        with self.state_lock:
            self._recovery_session = int(resp.session_id)
            self._phase_actions.clear()

    def start_sysid_sequence(self, prep_pose, max_limits):
        if self.sysid_calc is None:
            self._report_error("Friction identification engine is unavailable")
            return
        not_ready = False
        has_payload = False
        with self.state_lock:
            snap = self.supervisor_state
            if snap is not None and snap.payload_mass > 1e-6:
                has_payload = True
            elif (self._begin_pending or self._calculation_active or self._motion_pending or
                    snap is None or not snap.motion_allowed):
                not_ready = True
            else:
                self._set_sysid_preparation_status("Loading robot model...")
                self._prepare_epoch += 1
                epoch = self._prepare_epoch
                revision = snap.revision
        if has_payload:
            self._report_error(
                "Friction Identification is not allowed while payload mass > 0 kg.\n"
                "Unload the robot and set Payload Mass to 0.0 kg in Manual Control first.")
            return
        if not_ready:
            self._report_error("Supervisor or friction identification worker is not ready")
            return
        # At 1 ms the full trajectory has tens of thousands of points. Preflight
        # and ROS message construction must block neither Qt nor ROS callbacks.
        self._spawn_bg_thread(
            self._prepare_sysid, (epoch, revision, list(prep_pose), list(max_limits)))

    def _set_sysid_preparation_status(self, status):
        # Caller holds state_lock. This is local work feedback, not a robot mode.
        self.sysid_preparation_status = status
        self._begin_pending = bool(status)
        self.sysid_preparation_signal.emit()

    def _sysid_preparation_progress(self, epoch, status):
        with self.state_lock:
            if self._stopping or epoch != self._prepare_epoch:
                raise InterruptedError("Friction identification preparation canceled")
            self._set_sysid_preparation_status(status)

    def _prepare_sysid(self, epoch, revision, prep_pose, max_limits):
        try:
            started = time.monotonic()
            # Each run owns its numerical model and data, including canceled runs.
            engine = SysIdEngine()
            engine.generate_identification_trajectory(
                prep_pose, max_limits,
                progress=lambda status: self._sysid_preparation_progress(epoch, status))
            generated = time.monotonic()
            self._sysid_preparation_progress(epoch, "Building 1 ms trajectory message...")
            _, positions, velocities, accelerations = engine.trajectory_samples(prep_pose)
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            for tick, (q, dq, ddq) in enumerate(zip(positions, velocities, accelerations)):
                if tick % 1000 == 0:
                    with self.state_lock:
                        if self._stopping or epoch != self._prepare_epoch:
                            return
                point = JointTrajectoryPoint()
                point.positions, point.velocities, point.accelerations = q.tolist(), dq.tolist(), ddq.tolist()
                sec, nanosec = divmod(tick * 1_000_000, 1_000_000_000)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                trajectory.points.append(point)
            state_changed_err = False
            proceed_send = False
            with self.state_lock:
                snap = self.supervisor_state
                if self._stopping or epoch != self._prepare_epoch:
                    return
                if snap is None or snap.revision != revision or not snap.motion_allowed:
                    self._set_sysid_preparation_status("")
                    state_changed_err = True
                else:
                    self.sysid_calc = engine
                    self._prepared_trajectory = trajectory
                    self._sysid_session = 0
                    self._phase_actions.clear()
                    self.sysid_record_data.clear()
                    self.sysid_start_ros_time = None
                    self.sysid_q0_locked = prep_pose
                    self._set_sysid_preparation_status("Waiting for Supervisor acceptance...")
                    proceed_send = True
            if state_changed_err:
                self._report_error("System state changed during friction identification preparation; start again when ready")
                return
            if proceed_send:
                if not self._stopping:
                    self.node.get_logger().info(engine.trajectory_summary + '; reference period 1 ms')
                    self.node.get_logger().info(
                        f'Friction ID preparation: model/validation {generated - started:.3f} s, '
                        f'samples/message {time.monotonic() - generated:.3f} s, '
                        f'{len(trajectory.points)} points')
                if not self._send_command(SupervisorCommand.Request.CMD_BEGIN_SYSID,
                                          expected_revision=revision,
                                          done=lambda resp: self._begin_sysid_reply(resp, epoch)):
                    with self.state_lock:
                        if (not self._stopping and epoch == self._prepare_epoch and
                                self.sysid_preparation_status == "Waiting for Supervisor acceptance..."):
                            self._set_sysid_preparation_status("")
        except Exception as exc:
            with self.state_lock:
                if self._stopping or epoch != self._prepare_epoch:
                    return
                self._set_sysid_preparation_status("")
            self._report_error(f"Friction identification setup failed: {exc}")

    def _begin_sysid_reply(self, resp, epoch):
        with self.state_lock:
            if epoch != self._prepare_epoch or self._stopping:
                return
            if resp is None or not resp.accepted:
                self._set_sysid_preparation_status("")
                return
            self._sysid_session = int(resp.session_id)
            snap = self.supervisor_state
            if (snap is not None and snap.mode == SupervisorState.MODE_SYSID and
                    snap.session_id == self._sysid_session):
                self._set_sysid_preparation_status("")
            else:
                self._set_sysid_preparation_status("Waiting for Supervisor state...")
        if snap is not None:
            self._drive_supervisor_phase(snap)

    def _submit_sysid_trajectory(self, session):
        with self.state_lock:
            if not self._sysid_phase_is(session, SupervisorState.SYSID_READY_FOR_EXCITATION):
                return
            trajectory = self._prepared_trajectory
        if trajectory is None:
            self._report_error("Prepared friction identification trajectory unavailable")
            self._abort_sysid(session, "Excitation submission unavailable")
            return
        req = SubmitSysIdTrajectory.Request()
        req.session_id = int(session)
        req.trajectory = trajectory
        if not self._run_on_executor(lambda: self._submit_sysid_on_executor(session, req)):
            self._abort_sysid(session, "Excitation submission canceled")

    def _submit_sysid_on_executor(self, session, req):
        if not self._sysid_phase_is(session, SupervisorState.SYSID_READY_FOR_EXCITATION):
            return
        if not self.sysid_client.service_is_ready():
            self._report_error("Supervisor friction identification trajectory service unavailable")
            self._abort_sysid(session, "Excitation submission unavailable")
            return
        try:
            self.sysid_client.call_async(req).add_done_callback(
                lambda f: self._excitation_reply(f, session))
        except Exception as exc:
            self._report_error(f"Cannot send excitation trajectory: {exc}")
            self._abort_sysid(session, "Trajectory submission failed")

    def _sysid_phase_is(self, session, phase=None):
        with self.state_lock:
            state = self.supervisor_state
            return (not self._stopping and session == self._sysid_session and state is not None and
                    state.session_id == session and state.mode == SupervisorState.MODE_SYSID and
                    (phase is None or state.sysid_phase == phase))

    def _excitation_reply(self, future, session):
        if not self._sysid_phase_is(session):
            return
        try:
            resp = future.result()
            if not resp.accepted:
                self._report_error(f"Friction identification trajectory rejected: {resp.message}")
                self._abort_sysid(session, resp.message)
        except Exception as exc:
            self._report_error(f"Friction identification submission response failed: {exc}")
            self._abort_sysid(session, "No trajectory acceptance response")

    def run_sysid_calculation(self, session):
        with self.state_lock:
            if self._calculation_active or session != self._sysid_session:
                return
            self._calculation_active = True
            origin = self.sysid_start_ros_time
            engine = self.sysid_calc
            recorded = [dict(sample, t=sample['t'] - origin if origin is not None else sample['t'])
                        for sample in self.sysid_record_data]
        # Numerical estimation can be CPU-heavy; never block ROS heartbeat
        # subscriptions while waiting for the LS solve.
        self._spawn_bg_thread(self._calculate_sysid, (session, engine, recorded, origin))

    def _calculate_sysid(self, session, engine, recorded, origin):
        archive = None
        outcome = 'Calculation did not finish'
        try:
            engine.record_data = recorded
            # Archive before fitting, so malformed/incomplete experiments survive
            # quality-check failures. An I/O failure must not alter mode recovery.
            try:
                archive = engine.save_recording(session, origin)
                if not self._stopping:
                    self.node.get_logger().info(f'Friction ID raw recording: {archive}')
            except Exception as exc:
                if not self._stopping:
                    self.node.get_logger().warn(f'Could not archive friction ID recording: {exc}')
            if origin is None:
                raise ValueError("No CTC excitation start timestamp received")
            if not recorded:
                raise ValueError("No joint samples were recorded during excitation")
            result_text = engine.calculate_least_squares()
            outcome = result_text
            if archive is not None:
                result_text += f'\n# Raw recording: {archive}\n'
            if self._sysid_phase_is(session, SupervisorState.SYSID_CALCULATING):
                self.sysid_yaml_signal.emit(result_text)
                if not self._send_command(
                        SupervisorCommand.Request.CMD_SYSID_CALCULATION_DONE,
                        session=session, done=lambda resp: self._calculation_done_reply(resp, session)):
                    self._abort_sysid(session, "Cannot acknowledge calculation completion")
        except Exception as exc:
            outcome = str(exc)
            if self._sysid_phase_is(session, SupervisorState.SYSID_CALCULATING):
                # Data quality failure is not a motion failure. The Supervisor
                # performs the same guarded return sequence and retains the cause.
                archive_note = f'\n# Raw recording: {archive}' if archive is not None else ''
                self.sysid_yaml_signal.emit('# Calculation Failed: ' + str(exc).replace('\n', '\n# ') + archive_note)
                self._report_error(f"Friction identification failed; restoring NORMAL: {exc}")
                if not self._send_command(
                        SupervisorCommand.Request.CMD_SYSID_CALCULATION_FAILED,
                        session=session, reason='\n'.join(str(exc).splitlines()[:2])[:1500],
                        done=lambda resp: self._calculation_done_reply(resp, session)):
                    self._abort_sysid(session, "Cannot request controlled friction identification exit")
        finally:
            if archive is not None:
                try:
                    archive.with_suffix('.txt').write_text(outcome, encoding='utf-8')
                except OSError as exc:
                    if not self._stopping:
                        self.node.get_logger().warn(f'Could not save friction ID result: {exc}')
            with self.state_lock:
                self._calculation_active = False

    def _calculation_done_reply(self, resp, session):
        if resp is None or not resp.accepted:
            self._abort_sysid(session, "Supervisor denied calculation completion")

    def _abort_sysid(self, session, reason):
        with self.state_lock:
            s = self.supervisor_state
            if session == 0 or s is None or s.mode != SupervisorState.MODE_SYSID or s.session_id != session:
                return
        self._send_command(SupervisorCommand.Request.CMD_ABORT_SYSID,
                           session=session, reason=reason)

    def get_planner_error_str(self, code):
        messages = {
            IndustrialMotion.Result.SUCCESS: "Success",
            IndustrialMotion.Result.ERR_INVALID_INPUT: "Invalid Input Parameters",
            IndustrialMotion.Result.ERR_NO_IK_SOLUTION: "No Valid IK / Cartesian Approximation",
            IndustrialMotion.Result.ERR_LIMIT_VIOLATION: "Physical Limit / Collision Violation",
            IndustrialMotion.Result.ERR_SINGULARITY: "No Valid IK Branch (Singularity Encountered)",
            IndustrialMotion.Result.ERR_QUADRANT_JUMP: "Quadrant Jump Prevented",
            IndustrialMotion.Result.ERR_RUCKIG_FAILED: "Trajectory Generation Failed",
            IndustrialMotion.Result.ERR_EXECUTION_ABORTED: "Execution Aborted",
            IndustrialMotion.Result.ERR_CANCELLED: "Goal Cancelled",
            IndustrialMotion.Result.ERR_BUSY: "Supervisor Busy",
            IndustrialMotion.Result.ERR_NOT_READY: "System Not Ready",
            IndustrialMotion.Result.ERR_TIMEOUT: "Execution Timeout",
            IndustrialMotion.Result.ERR_SUPERVISOR_DENIED: "Supervisor Denied",
            IndustrialMotion.Result.ERR_INTERNAL: "Internal Failure",
        }
        return messages.get(code, f"Unknown ErrorCode ({code})")
