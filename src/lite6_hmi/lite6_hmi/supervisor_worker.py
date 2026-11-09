"""ROS 2 transport adapter for the Lite6 GUI.

The planner's Supervisor is the *only* owner of the global discrete state.
This module never transitions the global machine: it translates state snapshots
into the existing HMI labels and sends guarded commands to the Supervisor.

GUI widgets, their layouts and the SysIdEngine numerical methods live elsewhere.
"""

import math
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
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from unique_identifier_msgs.msg import UUID
from lite6_interfaces.action import IndustrialMotion
from lite6_interfaces.msg import SupervisorState, MotionRejection
from lite6_interfaces.srv import SupervisorCommand, SubmitSysIdTrajectory
from PyQt5.QtCore import QThread, pyqtSignal

from lite6_hmi.sysid_engine import SysIdEngine


class Lite6StateMachine:
    """Legacy *view labels*. These are NOT writable states or an FSM."""
    INIT = "INITIALIZING..."
    IDLE = "IDLE (Ready)"
    PLANNING = "PLANNING..."
    EXECUTING = "EXECUTING..."
    ERROR = "HARDWARE E-STOP!"
    RECOVERING = "RECOVERING..."
    SOFT_ERROR = "SOFTWARE FAULT"
    SYSID_PRE_ALIGN = "SYSID: PRE-ALIGNING..."
    SYSID_SEND_CMD = "SYSID: CONFIGURING HARDWARE..."
    SYSID_RUNNING = "SYSID: EXCITING TRAJECTORY..."
    SYSID_CALC = "SYSID: LEAST SQUARES SOLVING..."
    SYSID_POST_ALIGN = "SYSID: ALIGNING JTC TARGETS..."
    SYSID_RESTORE = "SYSID: RESTORING CLOSED-LOOP..."


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
    if msg.mode == S.MODE_SYSID:
        p = msg.sysid_phase
        labels = {
            S.SYSID_PRE_ALIGN: Lite6StateMachine.SYSID_PRE_ALIGN,
            S.SYSID_WAIT_CONTROLLER_MODE: Lite6StateMachine.SYSID_SEND_CMD,
            S.SYSID_READY_FOR_EXCITATION: "SYSID: READY FOR EXCITATION...",
            S.SYSID_EXCITING: Lite6StateMachine.SYSID_RUNNING,
            S.SYSID_CALCULATING: Lite6StateMachine.SYSID_CALC,
            S.SYSID_POST_ALIGN: Lite6StateMachine.SYSID_POST_ALIGN,
            S.SYSID_WAIT_NORMAL_MODE: Lite6StateMachine.SYSID_RESTORE,
            S.SYSID_RETURN_ALIGN: "SYSID: RETURNING TO ALIGNMENT POSE...",
        }
        return labels.get(p, "SYSID: WAITING...")
    if msg.motion_phase == S.MOTION_PLANNING:
        return Lite6StateMachine.PLANNING
    if msg.motion_phase == S.MOTION_WAITING_JTC:
        return "WAITING FOR JTC..."
    if msg.motion_phase == S.MOTION_EXECUTING:
        return Lite6StateMachine.EXECUTING
    if msg.motion_phase == S.MOTION_CANCELING:
        return "CANCELING MOTION..."
    return Lite6StateMachine.IDLE


class ROS2Worker(QThread):
    telemetry_signal = pyqtSignal(dict)
    state_signal = pyqtSignal(str, str)
    sysid_yaml_signal = pyqtSignal(str)
    command_error_signal = pyqtSignal(str)
    recovery_complete_signal = pyqtSignal()

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

        self.joint_names = [f"joint{i}" for i in range(1, 7)]
        self.q_curr = [0.0] * 6
        self.dq_curr = [0.0] * 6
        self.tau_curr = [0.0] * 6
        self.pose_curr = [0.0] * 6

        self.move_client = ActionClient(self.node, IndustrialMotion, "industrial_motion")
        self.command_client = self.node.create_client(
            SupervisorCommand, "/supervisor/command")
        self.sysid_client = self.node.create_client(
            SubmitSysIdTrajectory, "/supervisor/submit_sysid_trajectory")
        # Emergency fallback only. The normal mode (0) and SYS_ID mode (2)
        # are *never* commanded directly by the GUI.
        self.cmd_pub = self.node.create_publisher(
            Int32, "/ctc_controller/system_cmd", 10)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.supervisor_sub = self.node.create_subscription(
            SupervisorState, "/supervisor/state", self._supervisor_cb, qos)
        self.joint_sub = self.node.create_subscription(
            JointState, "/joint_states", self.joint_cb, 200)

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
        self._sysid_requested = False
        self._sysid_session = 0
        self._recovery_session = 0
        self._phase_actions = set()
        self._calculation_active = False
        self._estop_requested_at = 0.0
        self._direct_estop_sent = False

        self.sysid_record_data = []
        self.sysid_start_ros_time = None
        self.sysid_q0_locked = [0.0] * 6
        self._last_sysid_sample_time = None

        try:
            import os
            path = os.path.join(get_package_share_directory("lite6_description"),
                                "urdf", "lite6.urdf")
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

    def run(self):
        try:
            self.executor.spin()
        finally:
            self.executor.remove_node(self.node)

    def stop(self):
        self._stopping = True
        self.executor.shutdown(timeout_sec=2.0)
        self.wait(2500)
        self.node.destroy_node()

    def _report_error(self, message):
        self.node.get_logger().warn(message)
        if not self._stopping:
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

    def _send_command(self, event, session=0, reason="", done=None):
        if not self.command_client.service_is_ready():
            self._report_error("Supervisor command service is unavailable")
            return False
        req = SupervisorCommand.Request()
        req.command = event
        req.session_id = int(session)
        req.reason = str(reason)
        try:
            future = self.command_client.call_async(req)
            future.add_done_callback(lambda f: self._command_response(f, done))
            return True
        except Exception as exc:
            self._report_error(f"Supervisor command request failed: {exc}")
            return False

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
            # A stale latched / out-of-order message cannot roll back the view.
            if msg.revision < self._revision:
                return
            previous = self.supervisor_state
            self.last_supervisor_rx_time = now
            if msg.revision == self._revision and previous is not None:
                # Repeated heartbeats may update freshness flags but do not
                # represent new transitions. Store their latest projection.
                pass
            self._revision = msg.revision
            self.supervisor_state = msg
            self.state = _view_state(msg)
            self.error_msg = msg.detail if msg.mode in (
                SupervisorState.MODE_ESTOP, SupervisorState.MODE_FAULT,
                SupervisorState.MODE_SOFT_FAULT) else ""
            display = (self.state, self.error_msg)
            changed = display != self._last_view
            self._last_view = display
            if msg.mode == SupervisorState.MODE_ESTOP:
                self._estop_requested_at = 0.0
            # After an observed, completed recovery, another independent
            # emergency fallback must be possible in the next operating cycle.
            if msg.mode == SupervisorState.MODE_NORMAL and msg.controller_fresh:
                self._direct_estop_sent = False
            if msg.mode in (SupervisorState.MODE_ESTOP,
                            SupervisorState.MODE_FAULT):
                self._sysid_requested = False
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
                self._sysid_requested = False
                self._phase_actions.clear()
                self.node.get_logger().info("Supervisor confirmed SysID complete")
        if changed:
            self.state_signal.emit(display[0], display[1])
        self._drive_supervisor_phase(msg)
        self._drive_recovery_phase(msg)

    def _drive_supervisor_phase(self, msg):
        """Entry actions only; the Supervisor decides every next state."""
        with self.state_lock:
            sid = self._sysid_session
            if (not sid or msg.session_id != sid or
                    msg.mode != SupervisorState.MODE_SYSID):
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
            self._submit_sysid_trajectory(sid, prep)
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

    def _drive_recovery_phase(self, msg):
        with self.state_lock:
            sid = self._recovery_session
            if (sid == 0 or msg.session_id != sid or
                    msg.mode != SupervisorState.MODE_RECOVERING or
                    msg.recovery_phase != SupervisorState.RECOVERY_SYNC_REFERENCE):
                return
            key = (sid, "recovery-sync")
            if key in self._phase_actions:
                return
            self._phase_actions.add(key)
            q_now = list(self.q_curr)
        if not self.execute_joint_target(
                q_now, 0.1, 0.1,
                context=IndustrialMotion.Goal.CONTEXT_RECOVERY, session_id=sid):
            self.trigger_estop("Could not dispatch recovery reference synchronization")

    def joint_cb(self, msg):
        monotonic_now = time.monotonic()
        msg_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        index = {name: i for i, name in enumerate(msg.name)}
        # Don't treat incomplete / malformed packets as healthy telemetry.
        if any(name not in index or index[name] >= len(msg.position)
               for name in self.joint_names):
            return
        with self.state_lock:
            for i, name in enumerate(self.joint_names):
                j = index[name]
                self.q_curr[i] = float(msg.position[j])
                self.dq_curr[i] = float(msg.velocity[j]) if j < len(msg.velocity) else 0.0
                self.tau_curr[i] = float(msg.effort[j]) if j < len(msg.effort) else 0.0
            if not all(math.isfinite(x) for x in self.q_curr + self.dq_curr):
                return
            self.last_state_rx_time = monotonic_now
            s = self.supervisor_state
            # Data capture is enabled by a Supervisor-authored SysID phase,
            # not by local timers or manually assigned GUI state strings.
            if (self.sysid_calc is not None and s is not None and
                    s.mode == SupervisorState.MODE_SYSID and
                    s.sysid_phase == SupervisorState.SYSID_EXCITING and
                    s.motion_phase == SupervisorState.MOTION_EXECUTING and
                    s.session_id == self._sysid_session):
                if self.sysid_start_ros_time is None:
                    self.sysid_start_ros_time = msg_stamp
                elapsed = msg_stamp - self.sysid_start_ros_time
                if (self._last_sysid_sample_time is None or
                        msg_stamp > self._last_sysid_sample_time):
                    self._last_sysid_sample_time = msg_stamp
                    # Keep both stages and their transition samples for acceleration
                    # estimation. The engine applies separate guarded fit windows.
                    complete_effort = all(index[name] < len(msg.effort) and
                                          index[name] < len(msg.velocity)
                                          for name in self.joint_names)
                    if (0.0 <= elapsed <= self.sysid_calc.T_total and complete_effort and
                            all(math.isfinite(x) for x in self.tau_curr)):
                        self.sysid_record_data.append({
                            "t": elapsed,
                            "q": np.array(self.q_curr, dtype=float),
                            "dq": np.array(self.dq_curr, dtype=float),
                            "tau_cmd": np.array(self.tau_curr, dtype=float),
                        })
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
        with self.state_lock:
            snap = self.supervisor_state
            if self._motion_pending:
                self._report_error("A motion request is already awaiting an Action response")
                return False
            if goal.context == IndustrialMotion.Goal.CONTEXT_NORMAL:
                if snap is None or not snap.motion_allowed:
                    self._report_error("Supervisor does not currently permit manual motion")
                    return False
            elif snap is None or snap.session_id != goal.session_id:
                self._report_error("Session-scoped motion is not authorized")
                return False
            if not self.move_client.server_is_ready():
                self._report_error("IndustrialMotion Action server unavailable")
                return False
            self._motion_pending = True
            self._goal_epoch += 1
            epoch = self._goal_epoch
            goal_id = uuid.uuid4()
            key = goal_id.hex
            self._motion_requests[key] = {
                'created': time.monotonic(), 'rejected_at': None, 'detail': None}
        try:
            fut = self.move_client.send_goal_async(goal, goal_uuid=UUID(uuid=list(goal_id.bytes)))
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
        elif context == IndustrialMotion.Goal.CONTEXT_RECOVERY:
            self.trigger_estop("Recovery synchronization failed: " + reason)

    def _goal_response(self, future, epoch, context, session, key):
        try:
            handle = future.result()
            with self.state_lock:
                if epoch != self._goal_epoch:
                    self._motion_requests.pop(key, None)
                    if handle is not None and handle.accepted:
                        handle.cancel_goal_async()
                    return
                self._motion_pending = False
                if not handle.accepted:
                    request = self._motion_requests.setdefault(key, {
                        'created': time.monotonic(), 'detail': None})
                    request['rejected_at'] = time.monotonic()
                    self._flush_motion_rejections()
                    self._scoped_dispatch_failed(
                        context, session, "Supervisor rejected internal motion")
                    return
                self._motion_requests.pop(key, None)
                self.current_goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda f: self._goal_result(f, epoch, context, session))
        except Exception as exc:
            with self.state_lock:
                self._motion_requests.pop(key, None)
                if epoch == self._goal_epoch:
                    self._motion_pending = False
            self._report_error(f"Action goal response failed: {exc}")
            self._scoped_dispatch_failed(context, session, "Action response unavailable")

    def _goal_result(self, future, epoch, context, session):
        try:
            wrapped = future.result()
            with self.state_lock:
                if epoch != self._goal_epoch or epoch == self._result_reported_epoch:
                    return
                self._result_reported_epoch = epoch
                self.current_goal_handle = None
            result = wrapped.result
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
        """Request supervisor ESTOP; use CTC direct path only if unavailable."""
        with self.state_lock:
            self._estop_requested_at = time.monotonic()
            h = self.current_goal_handle
        if h is not None:
            try:
                h.cancel_goal_async()  # per-goal; Supervisor also preempts it
            except Exception:
                pass
        if force_fallback or not self.command_client.service_is_ready():
            self._emergency_fallback(reason)
            return
        ok = self._send_command(SupervisorCommand.Request.CMD_ESTOP, reason=reason,
                                done=self._estop_reply)
        if not ok:
            self._emergency_fallback(reason)

    def _estop_reply(self, resp):
        if resp is None or not resp.accepted:
            self._emergency_fallback("Supervisor did not accept emergency stop")

    def _emergency_fallback(self, reason):
        with self.state_lock:
            if self._direct_estop_sent:
                return
            self._direct_estop_sent = True
        self.cmd_pub.publish(Int32(data=1))
        self.node.get_logger().error(f"CTC DIRECT ESTOP FALLBACK: {reason}")

    def check_watchdog(self):
        """Qt 500 ms timer invokes this; no independent global FSM."""
        now = time.monotonic()
        with self.state_lock:
            s = self.supervisor_state
            age = now - self.last_supervisor_rx_time if self.last_supervisor_rx_time else float("inf")
            pending_stop = self._estop_requested_at
            already_fallback = self._direct_estop_sent
        if age > 1.2 and now - self.boot_time > 2.0:
            if not already_fallback:
                self.trigger_estop("SUPERVISOR HEARTBEAT LOST", force_fallback=True)
            return True
        if pending_stop and now - pending_stop > 0.6:
            if s is None or s.mode != SupervisorState.MODE_ESTOP:
                self._emergency_fallback("ESTOP not confirmed by supervisor")
        return False

    def begin_recovery(self):
        with self.state_lock:
            s = self.supervisor_state
            if s is None or s.mode != SupervisorState.MODE_ESTOP:
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
            snap = self.supervisor_state
            self._phase_actions.clear()
        if snap is not None:
            self._drive_recovery_phase(snap)

    def start_sysid_sequence(self, prep_pose, max_limits):
        if self.sysid_calc is None:
            self._report_error("System identification engine is unavailable")
            return
        with self.state_lock:
            snap = self.supervisor_state
            if self._begin_pending or snap is None or not snap.motion_allowed:
                self._report_error("Supervisor is not ready for system identification")
                return
            self._begin_pending = True
            self._sysid_requested = True
            self._sysid_session = 0
            self._phase_actions.clear()
            self.sysid_record_data.clear()
            self.sysid_start_ros_time = None
            self._last_sysid_sample_time = None
            self.sysid_q0_locked = list(prep_pose)
        # Prepare and preflight before changing controller mode.
        try:
            self.inject_sysid_parameters(prep_pose, max_limits)
        except Exception as exc:
            with self.state_lock:
                self._begin_pending = False
                self._sysid_requested = False
            self._report_error(f"System identification setup failed: {exc}")
            return
        if not self._send_command(SupervisorCommand.Request.CMD_BEGIN_SYSID,
                                  done=self._begin_sysid_reply):
            with self.state_lock:
                self._begin_pending = False
                self._sysid_requested = False

    def _begin_sysid_reply(self, resp):
        with self.state_lock:
            self._begin_pending = False
            if resp is None or not resp.accepted:
                self._sysid_requested = False
                return
            self._sysid_session = int(resp.session_id)
            snap = self.supervisor_state
        if snap is not None:
            self._drive_supervisor_phase(snap)

    def inject_sysid_parameters(self, prep_pose, max_limits):
        self.sysid_calc.generate_identification_trajectory(prep_pose, max_limits)
        self.node.get_logger().info(self.sysid_calc.trajectory_summary)

    def _submit_sysid_trajectory(self, session, prep):
        if not self.sysid_client.service_is_ready():
            self._report_error("Supervisor SysID trajectory service unavailable")
            self._abort_sysid(session, "Excitation service unavailable")
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for t in self.sysid_calc.trajectory_times():
            point = JointTrajectoryPoint()
            q, dq, ddq = self.sysid_calc.get_trajectory_point(t, prep)
            point.positions = q.tolist()
            point.velocities = dq.tolist()
            point.accelerations = ddq.tolist()
            # Integer nanoseconds: never emit nanosec == 1_000_000_000
            # due to floating-point rounding at whole seconds.
            sec, nanosec = divmod(int(round(t * 1_000_000_000)),
                                  1_000_000_000)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            trajectory.points.append(point)
        req = SubmitSysIdTrajectory.Request()
        req.session_id = int(session)
        req.trajectory = trajectory
        try:
            self.sysid_client.call_async(req).add_done_callback(
                lambda f: self._excitation_reply(f, session))
        except Exception as exc:
            self._report_error(f"Cannot send excitation trajectory: {exc}")
            self._abort_sysid(session, "Trajectory submission failed")

    def _excitation_reply(self, future, session):
        try:
            resp = future.result()
            if not resp.accepted:
                self._report_error(f"SysID trajectory rejected: {resp.message}")
                self._abort_sysid(session, resp.message)
        except Exception as exc:
            self._report_error(f"SysID submission response failed: {exc}")
            self._abort_sysid(session, "No trajectory acceptance response")

    def run_sysid_calculation(self, session):
        with self.state_lock:
            if self._calculation_active or session != self._sysid_session:
                return
            self._calculation_active = True
            recorded = list(self.sysid_record_data)
        # Numerical estimation can be CPU-heavy; never block ROS heartbeat
        # subscriptions while waiting for the LS solve.
        threading.Thread(target=self._calculate_sysid,
                         args=(session, recorded), daemon=True).start()

    def _calculate_sysid(self, session, recorded):
        try:
            if not recorded:
                raise ValueError("No joint samples were recorded during excitation")
            self.sysid_calc.record_data = recorded
            result_text = self.sysid_calc.calculate_least_squares()
            if result_text.lstrip().startswith("# Error") or result_text.lstrip().startswith("# Calculation Failed"):
                raise RuntimeError(result_text)
            with self.state_lock:
                valid = (session == self._sysid_session and
                         self.supervisor_state is not None and
                         self.supervisor_state.mode == SupervisorState.MODE_SYSID and
                         self.supervisor_state.sysid_phase == SupervisorState.SYSID_CALCULATING)
            if valid:
                self.sysid_yaml_signal.emit(result_text)
                self._send_command(
                    SupervisorCommand.Request.CMD_SYSID_CALCULATION_DONE,
                    session=session, done=lambda resp: self._calculation_done_reply(resp, session))
        except Exception as exc:
            self.sysid_yaml_signal.emit(f"# Calculation Failed: {exc}")
            self._report_error(f"SysID least-squares calculation failed: {exc}")
            self._abort_sysid(session, "Parameter estimation failed")
        finally:
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
