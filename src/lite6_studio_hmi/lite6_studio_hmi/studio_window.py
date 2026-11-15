"""Main PyQt5 IDE & Execution Monitoring Window for Lite6 Studio."""

from datetime import datetime
import math
import os

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from lite6_interfaces.msg import CompileDiagnostic, ControllerStatus, GripperState, StudioState
from lite6_interfaces.srv import GripperCommand, StudioCommand

from lite6_studio_hmi.code_editor import CodeEditor
from lite6_studio_hmi.dsl_examples import PRESET_PROGRAMS

_STATE_META = {
    StudioState.STATE_BOOT_INIT: ("BOOT INIT", "#4b5563", "#ffffff"),
    StudioState.STATE_IDLE: ("IDLE", "#334155", "#f8fafc"),
    StudioState.STATE_COMPILED_READY: ("COMPILED READY", "#0284c7", "#ffffff"),
    StudioState.STATE_RUNNING: ("RUNNING", "#16a34a", "#ffffff"),
    StudioState.STATE_PAUSED: ("PAUSED", "#d97706", "#ffffff"),
    StudioState.STATE_STEPPING: ("STEPPING", "#0d9488", "#ffffff"),
    StudioState.STATE_STOPPING: ("STOPPING", "#b45309", "#ffffff"),
    StudioState.STATE_ESTOP: ("E-STOP", "#dc2626", "#ffffff"),
    StudioState.STATE_FAULT: ("FAULT", "#991b1b", "#ffffff"),
    StudioState.STATE_RECOVERING: ("RECOVERING", "#7c3aed", "#ffffff"),
}

_CTRL_MODE_NAMES = {
    ControllerStatus.MODE_UNKNOWN: "UNKNOWN",
    ControllerStatus.MODE_NORMAL: "NORMAL (RIGID CTC)",
    ControllerStatus.MODE_ESTOP: "ESTOP",
    ControllerStatus.MODE_SYSID: "SYSID",
    ControllerStatus.MODE_JOINT_IMPEDANCE: "JOINT IMPEDANCE",
    ControllerStatus.MODE_CARTESIAN_IMPEDANCE: "CARTESIAN IMPEDANCE",
    ControllerStatus.MODE_TEACH: "TEACH",
    ControllerStatus.MODE_COMPLIANCE_STOPPING: "COMPLIANCE STOPPING",
    ControllerStatus.MODE_CARTESIAN_IMPEDANCE_TRAJECTORY: "IMPEDANCE TRAJECTORY",
}

_ESTOP_REASON_NAMES = {
    ControllerStatus.ESTOP_NONE: "None",
    ControllerStatus.ESTOP_EXTERNAL: "External / Operator E-STOP",
    ControllerStatus.ESTOP_COMPLIANCE_POSITION_LIMIT: "Compliance Joint Position Limit",
    ControllerStatus.ESTOP_COMPLIANCE_VELOCITY_LIMIT: "Compliance Joint Velocity Limit",
    ControllerStatus.ESTOP_COMPLIANCE_SATURATION: "Compliance Torque Saturation",
    ControllerStatus.ESTOP_TRAJECTORY_PATH_POSITION: "Rigid Path Position Tolerance",
    ControllerStatus.ESTOP_TRAJECTORY_PATH_VELOCITY: "Rigid Path Velocity Tolerance",
    ControllerStatus.ESTOP_TRAJECTORY_GOAL_TIMEOUT: "Rigid Goal Settle Timeout",
    ControllerStatus.ESTOP_TRAJECTORY_SATURATION: "Rigid Torque Saturation",
    ControllerStatus.ESTOP_TRAJECTORY_MODE_CHANGED: "Controller Mode Changed",
    ControllerStatus.ESTOP_INVALID_STATE: "Invalid / Non-finite Feedback",
    ControllerStatus.ESTOP_INVALID_COMMAND: "Invalid Torque Command",
    ControllerStatus.ESTOP_COMMAND_WRITE: "Hardware Command Write Error",
    ControllerStatus.ESTOP_CONTROL_EXCEPTION: "Controller Exception",
    ControllerStatus.ESTOP_IMPEDANCE_TRAJECTORY_DEVIATION: "Impedance Deviation Limit",
    ControllerStatus.ESTOP_IMPEDANCE_TRAJECTORY_GOAL_TIMEOUT: "Impedance Goal Timeout",
}

_GRIPPER_STATE_NAMES = {
    GripperState.BLOCKED: ("BLOCKED", "#991b1b"),
    GripperState.STOPPED: ("STOPPED", "#475569"),
    GripperState.OPENING: ("OPENING", "#0284c7"),
    GripperState.CLOSING: ("CLOSING", "#0284c7"),
    GripperState.OPEN: ("OPEN", "#16a34a"),
    GripperState.CLOSED: ("CLOSED", "#0f766e"),
    GripperState.GRASPED: ("GRASPED", "#059669"),
    GripperState.FAULT: ("FAULT", "#dc2626"),
}

_DIAG_STAGE_NAMES = {
    CompileDiagnostic.STAGE_LEX_SYNTAX: "Lex / Syntax",
    CompileDiagnostic.STAGE_SEMANTIC: "Semantic",
    CompileDiagnostic.STAGE_PLANNING: "Optimal Planning",
    CompileDiagnostic.STAGE_KINEMATIC_LIMIT: "Kinematic Limit",
    CompileDiagnostic.STAGE_COLLISION: "Collision Check",
}

_STYLESHEET = """
QMainWindow, QWidget#StudioRoot {
    background-color: #11161f;
    color: #e2e8f0;
    font-family: "Segoe UI", "Ubuntu", "Noto Sans", sans-serif;
    font-size: 9.5pt;
}
QGroupBox {
    background-color: #19202c;
    border: 1px solid #2b3547;
    border-radius: 6px;
    margin-top: 10px;
    padding: 8px;
    font-weight: bold;
    color: #cbd5e1;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    color: #93c5fd;
}
QPlainTextEdit {
    background-color: #151a24;
    color: #e5e7eb;
    border: 1px solid #2b3547;
    border-radius: 4px;
    selection-background-color: #264f78;
}
QTableWidget {
    background-color: #151a24;
    alternate-background-color: #1a2130;
    color: #e2e8f0;
    border: 1px solid #2b3547;
    gridline-color: #263042;
    selection-background-color: #1e3a5f;
}
QHeaderView::section {
    background-color: #1f2735;
    color: #94a3b8;
    border: 1px solid #2b3547;
    padding: 4px 6px;
    font-weight: bold;
}
QPushButton {
    background-color: #243042;
    color: #f1f5f9;
    border: 1px solid #38465c;
    border-radius: 4px;
    padding: 5px 11px;
    font-weight: 600;
}
QPushButton:hover:enabled {
    background-color: #2f3e55;
    border-color: #60a5fa;
}
QPushButton:pressed:enabled {
    background-color: #1d2838;
}
QPushButton:disabled {
    background-color: #181f2b;
    color: #526075;
    border-color: #222a38;
}
QComboBox {
    background-color: #1f2735;
    color: #e2e8f0;
    border: 1px solid #38465c;
    border-radius: 4px;
    padding: 4px 8px;
}
QProgressBar {
    background-color: #0f141c;
    border: 1px solid #2b3547;
    border-radius: 4px;
    text-align: center;
    color: #f8fafc;
    height: 18px;
}
QProgressBar::chunk {
    background-color: #0284c7;
    border-radius: 3px;
}
QSplitter::handle {
    background-color: #273142;
}
"""


def _styled_pill(bg_hex, fg_hex="#ffffff"):
    return (
        f"background-color: {bg_hex}; color: {fg_hex}; border-radius: 4px; "
        "padding: 3px 9px; font-weight: bold;"
    )


class StudioMainWindow(QMainWindow):
    """End-user industrial trajectory programming and execution Studio IDE."""

    def __init__(self, ros_worker, parent=None):
        super().__init__(parent)
        self.worker = ros_worker
        self.setWindowTitle("Lite6 Industrial Trajectory Studio")
        self.resize(1380, 860)
        self.setStyleSheet(_STYLESHEET)

        self._latest_studio_state = None
        self._latest_q = None
        self._latest_tcp = None
        self._current_file_path = None
        self._source_dirty = False
        self._compile_in_flight = False
        self._compiled_program_id = ""
        self._diagnostics = []
        self._last_logged_revision = None

        root = QWidget()
        root.setObjectName("StudioRoot")
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)

        main_layout.addWidget(self._build_header_bar())
        main_layout.addWidget(self._build_action_toolbar())

        # Main workspace splitter: Left IDE / Diagnostics, Right Live Telemetry & Gripper
        h_splitter = QSplitter(Qt.Horizontal)
        h_splitter.addWidget(self._build_left_ide_pane())
        h_splitter.addWidget(self._build_right_monitor_pane())
        h_splitter.setStretchFactor(0, 62)
        h_splitter.setStretchFactor(1, 38)
        main_layout.addWidget(h_splitter, 1)

        # Load default industrial sample cleanly
        first_key = next(iter(PRESET_PROGRAMS))
        self.editor.setPlainText(PRESET_PROGRAMS[first_key])
        self._mark_source_dirty(False)
        self.lbl_compile_summary.setText("Not compiled - click Compile to plan & validate")

        self._connect_signals()
        self._refresh_button_states()

    # ------------------------------------------------------------------ UI Builders

    def _build_header_bar(self):
        frame = QFrame()
        frame.setStyleSheet("background-color: #19202c; border: 1px solid #2b3547; border-radius: 6px;")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(10, 6, 10, 6)

        title = QLabel("LITE6 STUDIO")
        title.setFont(QFont("Segoe UI", 12, QFont.Bold))
        title.setStyleSheet("color: #60a5fa; border: none;")
        layout.addWidget(title)

        self.lbl_state_badge = QLabel("BOOT INIT")
        self.lbl_state_badge.setStyleSheet(_styled_pill("#4b5563"))
        layout.addWidget(self.lbl_state_badge)

        layout.addSpacing(8)
        self.lbl_joint_fresh = QLabel("JOINTS: STALE")
        self.lbl_joint_fresh.setStyleSheet(_styled_pill("#475569"))
        layout.addWidget(self.lbl_joint_fresh)

        self.lbl_ctrl_fresh = QLabel("CTRL: STALE")
        self.lbl_ctrl_fresh.setStyleSheet(_styled_pill("#475569"))
        layout.addWidget(self.lbl_ctrl_fresh)

        self.lbl_grip_fresh = QLabel("GRIPPER: OFFLINE")
        self.lbl_grip_fresh.setStyleSheet(_styled_pill("#475569"))
        layout.addWidget(self.lbl_grip_fresh)

        layout.addStretch(1)

        self.btn_estop = QPushButton("EMERGENCY STOP")
        self.btn_estop.setMinimumWidth(160)
        self.btn_estop.setStyleSheet(
            "QPushButton { background-color: #dc2626; color: #ffffff; border: 2px solid #fca5a5; "
            "font-weight: bold; font-size: 10.5pt; padding: 6px 14px; }"
            "QPushButton:hover { background-color: #ef4444; }"
        )
        layout.addWidget(self.btn_estop)
        return frame

    def _build_action_toolbar(self):
        frame = QFrame()
        frame.setStyleSheet("background-color: #161c27; border: 1px solid #263042; border-radius: 6px;")
        outer = QHBoxLayout(frame)
        outer.setContentsMargins(8, 5, 8, 5)
        outer.setSpacing(6)

        # File & snippet tools
        self.btn_new = QPushButton("New")
        self.btn_open = QPushButton("Open")
        self.btn_save = QPushButton("Save")
        outer.addWidget(self.btn_new)
        outer.addWidget(self.btn_open)
        outer.addWidget(self.btn_save)

        self.cmb_presets = QComboBox()
        for name in PRESET_PROGRAMS:
            self.cmb_presets.addItem(name)
        self.cmb_presets.setMaximumWidth(250)
        outer.addWidget(self.cmb_presets)

        self.btn_load_preset = QPushButton("Load Example")
        outer.addWidget(self.btn_load_preset)

        self.btn_insert_joints = QPushButton("+ Joints")
        self.btn_insert_joints.setToolTip("Insert live measured joint vector into editor")
        self.btn_insert_pose = QPushButton("+ TCP Pose")
        self.btn_insert_pose.setToolTip("Insert live measured TCP Pose(...) into editor")
        outer.addWidget(self.btn_insert_joints)
        outer.addWidget(self.btn_insert_pose)

        self.chk_use_current_start = QCheckBox("Use Live Start Pose")
        self.chk_use_current_start.setChecked(True)
        self.chk_use_current_start.setToolTip(
            "When checked, compiles starting at measured /joint_states; uncheck for zero-pose offline dry-run."
        )
        outer.addWidget(self.chk_use_current_start)

        outer.addStretch(1)

        # Motion & state machine execution controls
        self.btn_compile = QPushButton("Compile")
        self.btn_compile.setStyleSheet(
            "QPushButton { background-color: #1d4ed8; border-color: #3b82f6; }"
            "QPushButton:hover:enabled { background-color: #2563eb; }"
        )
        self.btn_run_resume = QPushButton("Run")
        self.btn_run_resume.setStyleSheet(
            "QPushButton { background-color: #15803d; border-color: #22c55e; }"
            "QPushButton:hover:enabled { background-color: #16a34a; }"
        )
        self.btn_pause = QPushButton("Pause")
        self.btn_step = QPushButton("Step")
        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setStyleSheet(
            "QPushButton { background-color: #b45309; border-color: #f59e0b; }"
            "QPushButton:hover:enabled { background-color: #d97706; }"
        )
        self.btn_reset_fault = QPushButton("Reset Fault")
        self.btn_recover = QPushButton("Recover E-STOP")
        self.btn_recover.setStyleSheet(
            "QPushButton { background-color: #6d28d9; border-color: #a78bfa; }"
            "QPushButton:hover:enabled { background-color: #7c3aed; }"
        )
        self.btn_clear = QPushButton("Clear")

        for btn in (
            self.btn_compile,
            self.btn_run_resume,
            self.btn_pause,
            self.btn_step,
            self.btn_stop,
            self.btn_reset_fault,
            self.btn_recover,
            self.btn_clear,
        ):
            outer.addWidget(btn)

        return frame

    def _build_left_ide_pane(self):
        v_splitter = QSplitter(Qt.Vertical)

        # Top: Editor card with compile summary strip
        editor_container = QWidget()
        e_layout = QVBoxLayout(editor_container)
        e_layout.setContentsMargins(0, 0, 0, 0)
        e_layout.setSpacing(4)

        banner = QFrame()
        banner.setStyleSheet("background-color: #19202c; border: 1px solid #2b3547; border-radius: 4px;")
        b_layout = QHBoxLayout(banner)
        b_layout.setContentsMargins(8, 4, 8, 4)
        self.lbl_compile_summary = QLabel("Ready")
        self.lbl_compile_summary.setStyleSheet("border: none; color: #cbd5e1; font-weight: 600;")
        self.lbl_dirty_flag = QLabel("")
        self.lbl_dirty_flag.setStyleSheet("border: none; color: #f59e0b; font-weight: bold;")
        b_layout.addWidget(self.lbl_compile_summary, 1)
        b_layout.addWidget(self.lbl_dirty_flag)
        e_layout.addWidget(banner)

        self.editor = CodeEditor()
        e_layout.addWidget(self.editor, 1)
        v_splitter.addWidget(editor_container)

        # Bottom: Diagnostics panel
        diag_group = QGroupBox("Compiler Diagnostics (click row to jump to line)")
        d_layout = QVBoxLayout(diag_group)
        d_layout.setContentsMargins(6, 12, 6, 6)

        self.diag_table = QTableWidget(0, 6)
        self.diag_table.setHorizontalHeaderLabels(
            ["Severity", "Stage", "Line", "Col", "Code", "Diagnostic Message"]
        )
        self.diag_table.setAlternatingRowColors(True)
        self.diag_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.diag_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.diag_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.diag_table.verticalHeader().setVisible(False)
        hdr = self.diag_table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(5, QHeaderView.Stretch)
        d_layout.addWidget(self.diag_table)

        v_splitter.addWidget(diag_group)
        v_splitter.setStretchFactor(0, 72)
        v_splitter.setStretchFactor(1, 28)
        return v_splitter

    def _build_right_monitor_pane(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(4, 0, 2, 0)
        layout.setSpacing(6)

        # 1. Execution progress card
        prog_box = QGroupBox("Program Execution & Progress")
        p_grid = QGridLayout(prog_box)
        p_grid.setSpacing(5)

        self.lbl_prog_id = QLabel("--")
        self.lbl_block_idx = QLabel("0 / 0")
        self.lbl_active_line = QLabel("--")
        self.lbl_exec_detail = QLabel("--")
        self.lbl_exec_detail.setWordWrap(True)

        self.bar_block_progress = QProgressBar()
        self.bar_block_progress.setRange(0, 100)
        self.bar_total_progress = QProgressBar()
        self.bar_total_progress.setRange(0, 100)

        p_grid.addWidget(QLabel("Program ID:"), 0, 0)
        p_grid.addWidget(self.lbl_prog_id, 0, 1)
        p_grid.addWidget(QLabel("Block:"), 0, 2)
        p_grid.addWidget(self.lbl_block_idx, 0, 3)
        p_grid.addWidget(QLabel("Active Line:"), 1, 0)
        p_grid.addWidget(self.lbl_active_line, 1, 1)
        p_grid.addWidget(QLabel("Block %:"), 2, 0)
        p_grid.addWidget(self.bar_block_progress, 2, 1, 1, 3)
        p_grid.addWidget(QLabel("Total %:"), 3, 0)
        p_grid.addWidget(self.bar_total_progress, 3, 1, 1, 3)
        p_grid.addWidget(QLabel("Status:"), 4, 0)
        p_grid.addWidget(self.lbl_exec_detail, 4, 1, 1, 3)
        layout.addWidget(prog_box)

        # 2. Controller & Impedance Telemetry Card
        ctrl_box = QGroupBox("Controller & Impedance Tracking")
        c_grid = QGridLayout(ctrl_box)
        c_grid.setSpacing(5)

        self.lbl_ctrl_mode = QLabel("UNKNOWN")
        self.lbl_ctrl_mode.setStyleSheet("color: #93c5fd; font-weight: bold;")
        self.lbl_estop_reason = QLabel("None")
        self.lbl_estop_reason.setWordWrap(True)
        self.lbl_pos_err = QLabel("0.00 mm  [0.00, 0.00, 0.00]")
        self.lbl_rot_err = QLabel("0.00 deg [0.00, 0.00, 0.00]")

        c_grid.addWidget(QLabel("CTC Mode:"), 0, 0)
        c_grid.addWidget(self.lbl_ctrl_mode, 0, 1)
        c_grid.addWidget(QLabel("E-STOP / Fault:"), 1, 0)
        c_grid.addWidget(self.lbl_estop_reason, 1, 1)
        c_grid.addWidget(QLabel("Impedance Pos Err:"), 2, 0)
        c_grid.addWidget(self.lbl_pos_err, 2, 1)
        c_grid.addWidget(QLabel("Impedance Rot Err:"), 3, 0)
        c_grid.addWidget(self.lbl_rot_err, 3, 1)
        layout.addWidget(ctrl_box)

        # 3. Live Robot Joint & TCP Telemetry Card
        rob_box = QGroupBox("Live Robot Feedback")
        r_layout = QVBoxLayout(rob_box)
        r_layout.setSpacing(4)

        self.lbl_tcp_pose = QLabel("TCP: awaiting TF link_base -> link_eef...")
        self.lbl_tcp_pose.setStyleSheet("font-family: Consolas, Monospace; color: #38bdf8;")
        r_layout.addWidget(self.lbl_tcp_pose)

        j_grid = QGridLayout()
        j_grid.setSpacing(3)
        self._joint_labels = []
        for idx in range(6):
            row = idx // 2
            col_base = (idx % 2) * 2
            name_lbl = QLabel(f"J{idx + 1}:")
            name_lbl.setStyleSheet("color: #94a3b8; font-weight: bold;")
            val_lbl = QLabel("  0.000 rad (  0.0 deg)")
            val_lbl.setStyleSheet("font-family: Consolas, Monospace;")
            j_grid.addWidget(name_lbl, row, col_base)
            j_grid.addWidget(val_lbl, row, col_base + 1)
            self._joint_labels.append(val_lbl)
        r_layout.addLayout(j_grid)
        layout.addWidget(rob_box)

        # 4. Standalone Decoupled Gripper Card
        grip_box = QGroupBox("End-Effector (Decoupled Gripper Control)")
        g_layout = QVBoxLayout(grip_box)
        g_layout.setSpacing(5)

        g_info = QHBoxLayout()
        self.lbl_gripper_state = QLabel("OFFLINE")
        self.lbl_gripper_state.setStyleSheet(_styled_pill("#475569"))
        self.lbl_gripper_metrics = QLabel("Pos: -- mm | Effort: -- N")
        self.lbl_gripper_metrics.setStyleSheet("font-family: Consolas, Monospace;")
        g_info.addWidget(self.lbl_gripper_state)
        g_info.addWidget(self.lbl_gripper_metrics, 1)
        g_layout.addLayout(g_info)

        g_btns = QHBoxLayout()
        self.btn_grip_open = QPushButton("Open Gripper")
        self.btn_grip_close = QPushButton("Close Gripper")
        self.btn_grip_stop = QPushButton("Stop Gripper")
        self.btn_grip_open.setStyleSheet("QPushButton { background-color: #0369a1; }")
        self.btn_grip_close.setStyleSheet("QPushButton { background-color: #0f766e; }")
        g_btns.addWidget(self.btn_grip_open)
        g_btns.addWidget(self.btn_grip_close)
        g_btns.addWidget(self.btn_grip_stop)
        g_layout.addLayout(g_btns)
        layout.addWidget(grip_box)

        # 5. Event Console Log
        log_box = QGroupBox("Studio Event Log")
        l_layout = QVBoxLayout(log_box)
        l_layout.setContentsMargins(6, 12, 6, 6)
        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(300)
        self.txt_log.setMinimumHeight(120)
        self.txt_log.setFont(QFont("Consolas, Monospace", 9))
        l_layout.addWidget(self.txt_log)
        layout.addWidget(log_box, 1)

        scroll.setWidget(panel)
        return scroll

    # ------------------------------------------------------------------ Signals

    def _connect_signals(self):
        self.editor.textChanged.connect(self._on_editor_text_changed)
        self.diag_table.cellClicked.connect(self._on_diag_clicked)
        self.diag_table.cellDoubleClicked.connect(self._on_diag_clicked)

        self.btn_new.clicked.connect(self._on_new_script)
        self.btn_open.clicked.connect(self._on_open_script)
        self.btn_save.clicked.connect(self._on_save_script)
        self.btn_load_preset.clicked.connect(self._on_load_preset)
        self.btn_insert_joints.clicked.connect(self._on_insert_joints)
        self.btn_insert_pose.clicked.connect(self._on_insert_pose)

        self.btn_compile.clicked.connect(self.trigger_compile)
        self.btn_run_resume.clicked.connect(self._on_run_or_resume)
        self.btn_pause.clicked.connect(
            lambda: self.worker.request_command(StudioCommand.Request.CMD_PAUSE)
        )
        self.btn_step.clicked.connect(
            lambda: self.worker.request_command(StudioCommand.Request.CMD_STEP)
        )
        self.btn_stop.clicked.connect(
            lambda: self.worker.request_command(StudioCommand.Request.CMD_STOP)
        )
        self.btn_reset_fault.clicked.connect(
            lambda: self.worker.request_command(StudioCommand.Request.CMD_RESET_FAULT)
        )
        self.btn_recover.clicked.connect(
            lambda: self.worker.request_command(StudioCommand.Request.CMD_RECOVER)
        )
        self.btn_clear.clicked.connect(
            lambda: self.worker.request_command(StudioCommand.Request.CMD_CLEAR_PROGRAM)
        )
        self.btn_estop.clicked.connect(
            lambda: self.worker.request_command(
                StudioCommand.Request.CMD_ESTOP, "UI Emergency Stop button"
            )
        )

        self.btn_grip_open.clicked.connect(
            lambda: self.worker.request_gripper(GripperCommand.Request.OPEN)
        )
        self.btn_grip_close.clicked.connect(
            lambda: self.worker.request_gripper(GripperCommand.Request.CLOSE)
        )
        self.btn_grip_stop.clicked.connect(
            lambda: self.worker.request_gripper(GripperCommand.Request.STOP)
        )

        self.worker.studio_state_updated.connect(self.on_studio_state)
        self.worker.gripper_state_updated.connect(self.on_gripper_state)
        self.worker.telemetry_updated.connect(self.on_telemetry)
        self.worker.compile_finished.connect(self.on_compile_finished)
        self.worker.command_finished.connect(self.on_command_finished)
        self.worker.gripper_finished.connect(self.on_gripper_finished)

    # ------------------------------------------------------------------ Helpers

    def append_log(self, text):
        ts = datetime.now().strftime("%H:%M:%S")
        self.txt_log.appendPlainText(f"[{ts}] {text}")

    def _mark_source_dirty(self, dirty):
        self._source_dirty = bool(dirty)
        self.lbl_dirty_flag.setText("UNCOMPILED CHANGES" if self._source_dirty else "")
        self._refresh_button_states()

    def _current_state_code(self):
        if self._latest_studio_state is None:
            return StudioState.STATE_BOOT_INIT
        return int(self._latest_studio_state.state)

    def _refresh_button_states(self):
        st = self._current_state_code()
        online = self._latest_studio_state is not None

        can_compile = (not self._compile_in_flight) and st in (
            StudioState.STATE_IDLE,
            StudioState.STATE_COMPILED_READY,
            StudioState.STATE_PAUSED,
            StudioState.STATE_FAULT,
            StudioState.STATE_ESTOP,
        )
        # Keep editor read-only while program is actively executing or paused mid-run
        active_exec = st in (
            StudioState.STATE_RUNNING,
            StudioState.STATE_STEPPING,
            StudioState.STATE_PAUSED,
            StudioState.STATE_STOPPING,
            StudioState.STATE_RECOVERING,
        )
        self.editor.setReadOnly(active_exec)
        self.btn_compile.setEnabled(can_compile if online else True)

        if st == StudioState.STATE_PAUSED:
            self.btn_run_resume.setText("Resume")
            self.btn_run_resume.setEnabled(True)
            self.btn_step.setEnabled(True)
        elif st == StudioState.STATE_COMPILED_READY:
            self.btn_run_resume.setText("Run")
            ready_to_run = not self._source_dirty
            self.btn_run_resume.setEnabled(ready_to_run)
            self.btn_step.setEnabled(ready_to_run)
            self.btn_run_resume.setToolTip(
                "Execute compiled program"
                if ready_to_run
                else "Source edited: click Compile before running"
            )
        else:
            self.btn_run_resume.setText("Run")
            self.btn_run_resume.setEnabled(False)
            self.btn_step.setEnabled(False)

        self.btn_pause.setEnabled(st in (StudioState.STATE_RUNNING, StudioState.STATE_STEPPING))
        self.btn_stop.setEnabled(
            st in (StudioState.STATE_RUNNING, StudioState.STATE_STEPPING, StudioState.STATE_PAUSED)
        )
        self.btn_reset_fault.setEnabled(st == StudioState.STATE_FAULT)
        self.btn_recover.setEnabled(st == StudioState.STATE_ESTOP)
        self.btn_clear.setEnabled(
            st
            in (
                StudioState.STATE_COMPILED_READY,
                StudioState.STATE_PAUSED,
                StudioState.STATE_FAULT,
                StudioState.STATE_ESTOP,
            )
        )

    # ------------------------------------------------------------------ Actions

    def _on_editor_text_changed(self):
        if not self._source_dirty:
            self._mark_source_dirty(True)

    def _on_new_script(self):
        self.editor.setPlainText("# New Lite6Script program\nMoveJ([0.0, 0.35, 1.0, 0.0, 0.65, 0.0], vel=0.4, acc=0.4)\n")
        self._current_file_path = None
        self.editor.set_diagnostic_markers([], [])
        self.diag_table.setRowCount(0)
        self._mark_source_dirty(True)

    def _on_open_script(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Lite6Script Program", "", "Lite6Script (*.l6 *.txt);;All Files (*)"
        )
        if not path:
            return
        with open(path, "r", encoding="utf-8") as fh:
            self.editor.setPlainText(fh.read())
        self._current_file_path = path
        self.append_log(f"Opened {os.path.basename(path)}")

    def _on_save_script(self):
        path = self._current_file_path
        if not path:
            path, _ = QFileDialog.getSaveFileName(
                self, "Save Lite6Script Program", "program.l6", "Lite6Script (*.l6);;All Files (*)"
            )
            if not path:
                return
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(self.editor.toPlainText())
        self._current_file_path = path
        self.append_log(f"Saved {os.path.basename(path)}")

    def _on_load_preset(self):
        key = self.cmb_presets.currentText()
        if key in PRESET_PROGRAMS:
            self.editor.setPlainText(PRESET_PROGRAMS[key])
            self.editor.set_diagnostic_markers([], [])
            self.diag_table.setRowCount(0)
            self._mark_source_dirty(True)
            self.append_log(f"Loaded preset: {key}")

    def _on_insert_joints(self):
        if self._latest_q is None:
            QMessageBox.information(self, "No Joint State", "Waiting for fresh /joint_states.")
            return
        q_str = ", ".join(f"{v:.4f}" for v in self._latest_q)
        self.editor.insert_snippet_at_cursor(f"MoveJ([{q_str}], vel=0.50, acc=0.50)\n")

    def _on_insert_pose(self):
        if self._latest_tcp is None:
            QMessageBox.information(self, "No TCP TF", "Waiting for link_base -> link_eef TF.")
            return
        x, y, z, r, p, w = self._latest_tcp
        snippet = (
            f"MoveL(Pose(xyz=[{x:.5f}, {y:.5f}, {z:.5f}], "
            f"rpy=[{r:.5f}, {p:.5f}, {w:.5f}]), vel=0.40, acc=0.40)\n"
        )
        self.editor.insert_snippet_at_cursor(snippet)

    def trigger_compile(self):
        self._compile_in_flight = True
        self.lbl_compile_summary.setText("Compiling & validating 1 kHz optimal trajectory...")
        self.lbl_compile_summary.setStyleSheet("border: none; color: #60a5fa; font-weight: 600;")
        self._refresh_button_states()
        self.worker.request_compile(
            self.editor.toPlainText(),
            use_current_robot_start=self.chk_use_current_start.isChecked(),
            custom_start=[0.0] * 6,
        )

    def _on_run_or_resume(self):
        st = self._current_state_code()
        if st == StudioState.STATE_PAUSED:
            self.worker.request_command(StudioCommand.Request.CMD_RESUME)
        else:
            self.worker.request_command(StudioCommand.Request.CMD_RUN)

    def _on_diag_clicked(self, row, _col):
        if 0 <= row < len(self._diagnostics):
            d = self._diagnostics[row]
            self.editor.jump_to_line(max(1, d.line_number), max(1, d.column_number))

    # ------------------------------------------------------------------ ROS Callbacks

    def on_compile_finished(self, resp):
        self._compile_in_flight = False
        self._diagnostics = list(resp.diagnostics)
        self.diag_table.setRowCount(0)

        err_lines = set()
        warn_lines = set()
        for d in self._diagnostics:
            row = self.diag_table.rowCount()
            self.diag_table.insertRow(row)
            is_err = d.severity == CompileDiagnostic.SEVERITY_ERROR
            if is_err and d.line_number > 0:
                err_lines.add(int(d.line_number))
            elif d.line_number > 0:
                warn_lines.add(int(d.line_number))

            sev_item = QTableWidgetItem("ERROR" if is_err else "WARN")
            sev_item.setForeground(QColor("#f87171" if is_err else "#fbbf24"))
            stage_item = QTableWidgetItem(_DIAG_STAGE_NAMES.get(d.stage, str(d.stage)))
            line_item = QTableWidgetItem(str(d.line_number))
            col_item = QTableWidgetItem(str(d.column_number))
            code_item = QTableWidgetItem(d.code)
            msg_item = QTableWidgetItem(d.message)

            for col_idx, item in enumerate(
                (sev_item, stage_item, line_item, col_item, code_item, msg_item)
            ):
                self.diag_table.setItem(row, col_idx, item)

        self.editor.set_diagnostic_markers(err_lines, warn_lines)

        if resp.success:
            self._compiled_program_id = resp.program_id
            self._mark_source_dirty(False)
            summary = (
                f"Compile OK | Statements: {resp.total_statements} | "
                f"Blocks: {resp.total_blocks} | Est. Time: {resp.estimated_duration_sec:.2f} s | "
                f"1 kHz Samples: {resp.total_samples_1khz}"
            )
            self.lbl_compile_summary.setText(summary)
            self.lbl_compile_summary.setStyleSheet("border: none; color: #4ade80; font-weight: 600;")
            self.append_log(summary)
        else:
            err_count = sum(1 for d in self._diagnostics if d.severity == CompileDiagnostic.SEVERITY_ERROR)
            summary = f"Compile FAILED ({err_count} error(s)) - click diagnostic row to locate"
            self.lbl_compile_summary.setText(summary)
            self.lbl_compile_summary.setStyleSheet("border: none; color: #f87171; font-weight: 600;")
            self.append_log(summary)
            if self._diagnostics:
                self.diag_table.selectRow(0)
                first = self._diagnostics[0]
                self.editor.jump_to_line(max(1, first.line_number), max(1, first.column_number))

        self._refresh_button_states()

    def on_command_finished(self, cmd_code, accepted, message):
        if not accepted:
            self.append_log(f"Command {cmd_code} rejected: {message}")
        elif message:
            self.append_log(f"Command {cmd_code}: {message}")

    def on_gripper_finished(self, action_code, accepted, detail):
        tag = {1: "OPEN", 2: "CLOSE", 3: "STOP"}.get(action_code, str(action_code))
        status = "queued" if accepted else "rejected"
        self.append_log(f"Gripper {tag} {status}: {detail}")

    def on_studio_state(self, msg):
        self._latest_studio_state = msg

        name, bg, fg = _STATE_META.get(msg.state, (f"STATE_{msg.state}", "#475569", "#ffffff"))
        self.lbl_state_badge.setText(name)
        self.lbl_state_badge.setStyleSheet(_styled_pill(bg, fg))

        self.lbl_joint_fresh.setText(
            "JOINTS: FRESH" if msg.joint_states_fresh else "JOINTS: STALE"
        )
        self.lbl_joint_fresh.setStyleSheet(
            _styled_pill("#15803d" if msg.joint_states_fresh else "#b91c1c")
        )
        self.lbl_ctrl_fresh.setText("CTRL: FRESH" if msg.controller_fresh else "CTRL: STALE")
        self.lbl_ctrl_fresh.setStyleSheet(
            _styled_pill("#15803d" if msg.controller_fresh else "#b91c1c")
        )
        self.lbl_grip_fresh.setText("GRIPPER: READY" if msg.gripper_ready else "GRIPPER: NOT READY")
        self.lbl_grip_fresh.setStyleSheet(
            _styled_pill("#0f766e" if msg.gripper_ready else "#64748b")
        )

        self.lbl_prog_id.setText(msg.loaded_program_id if msg.loaded_program_id else "--")
        self.lbl_block_idx.setText(f"{msg.current_block_index} / {msg.total_blocks}")
        self.lbl_active_line.setText(
            f"Line {msg.current_line_number} (Stmt {msg.current_statement_index})"
            if msg.current_line_number > 0
            else "--"
        )
        self.bar_block_progress.setValue(int(round(max(0.0, min(100.0, msg.block_progress_percent)))))
        self.bar_total_progress.setValue(int(round(max(0.0, min(100.0, msg.total_progress_percent)))))
        self.lbl_exec_detail.setText(msg.detail if msg.detail else "--")

        # Highlight live executing line during RUNNING/PAUSED/STEPPING
        if msg.state in (
            StudioState.STATE_RUNNING,
            StudioState.STATE_PAUSED,
            StudioState.STATE_STEPPING,
        ):
            self.editor.set_active_line(msg.current_line_number)
        else:
            self.editor.set_active_line(0)

        ctrl_name = _CTRL_MODE_NAMES.get(msg.controller_mode, f"MODE_{msg.controller_mode}")
        self.lbl_ctrl_mode.setText(ctrl_name)
        if msg.estop_latched:
            reason_str = _ESTOP_REASON_NAMES.get(msg.estop_reason, f"ESTOP_{msg.estop_reason}")
            self.lbl_estop_reason.setText(f"ESTOP LATCHED: {reason_str}")
            self.lbl_estop_reason.setStyleSheet("color: #f87171; font-weight: bold;")
        elif msg.last_error_code != 0:
            self.lbl_estop_reason.setText(f"Fault code {msg.last_error_code}")
            self.lbl_estop_reason.setStyleSheet("color: #fbbf24; font-weight: bold;")
        else:
            self.lbl_estop_reason.setText("None")
            self.lbl_estop_reason.setStyleSheet("color: #94a3b8;")

        pe = msg.cartesian_pos_error_m
        re = msg.cartesian_rot_error_rad
        p_norm_mm = math.sqrt(pe[0] ** 2 + pe[1] ** 2 + pe[2] ** 2) * 1000.0
        r_norm_deg = math.degrees(math.sqrt(re[0] ** 2 + re[1] ** 2 + re[2] ** 2))
        self.lbl_pos_err.setText(
            f"{p_norm_mm:5.2f} mm  [{pe[0]*1000:5.1f}, {pe[1]*1000:5.1f}, {pe[2]*1000:5.1f}]"
        )
        self.lbl_rot_err.setText(
            f"{r_norm_deg:5.2f} deg [{math.degrees(re[0]):5.1f}, {math.degrees(re[1]):5.1f}, {math.degrees(re[2]):5.1f}]"
        )

        if msg.revision != self._last_logged_revision and msg.detail:
            self._last_logged_revision = msg.revision
            self.append_log(f"[{name}] {msg.detail}")

        if (
            msg.state == StudioState.STATE_IDLE
            and not msg.loaded_program_id
            and not self._compile_in_flight
            and self._compiled_program_id
        ):
            self._compiled_program_id = ""
            self.lbl_compile_summary.setText("Program cleared - click Compile to build")
            self.lbl_compile_summary.setStyleSheet("border: none; color: #cbd5e1; font-weight: 600;")

        self._refresh_button_states()

    def on_gripper_state(self, msg):
        gname, gbg = _GRIPPER_STATE_NAMES.get(msg.state, (f"STATE_{msg.state}", "#475569"))
        self.lbl_gripper_state.setText(gname)
        self.lbl_gripper_state.setStyleSheet(_styled_pill(gbg))
        pos_mm = (msg.position[0] - msg.position[1]) * 1000.0
        self.lbl_gripper_metrics.setText(f"Opening: {pos_mm:5.1f} mm | Effort: {msg.effort:5.1f} N")

    def on_telemetry(self, q_list, _dq_list, tcp6):
        if q_list is not None:
            self._latest_q = q_list
            for idx, val in enumerate(q_list):
                self._joint_labels[idx].setText(f"{val:7.3f} rad ({math.degrees(val):6.1f} deg)")
        if tcp6 is not None:
            self._latest_tcp = tcp6
            x, y, z, r, p, w = tcp6
            self.lbl_tcp_pose.setText(
                f"TCP xyz=[{x:6.4f}, {y:6.4f}, {z:6.4f}] m | rpy=[{r:6.3f}, {p:6.3f}, {w:6.3f}] rad"
            )
