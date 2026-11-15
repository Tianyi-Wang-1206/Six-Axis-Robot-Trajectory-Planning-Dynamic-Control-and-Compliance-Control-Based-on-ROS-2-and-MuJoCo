"""Main PyQt5 IDE & Execution Monitoring Window for Robot Studio (Light Theme)."""

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
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
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

_UI_FONT_FAMILY = "Noto Sans"
_MONO_FONT_FAMILY = "Noto Sans Mono"

_STATE_META = {
    StudioState.STATE_BOOT_INIT: ("BOOT INIT", "#64748B", "#FFFFFF"),
    StudioState.STATE_IDLE: ("IDLE", "#475569", "#FFFFFF"),
    StudioState.STATE_COMPILED_READY: ("COMPILED READY", "#0284C7", "#FFFFFF"),
    StudioState.STATE_RUNNING: ("RUNNING", "#16A34A", "#FFFFFF"),
    StudioState.STATE_PAUSED: ("PAUSED", "#D97706", "#FFFFFF"),
    StudioState.STATE_STEPPING: ("STEPPING", "#0D9488", "#FFFFFF"),
    StudioState.STATE_STOPPING: ("STOPPING", "#C2410C", "#FFFFFF"),
    StudioState.STATE_ESTOP: ("E-STOP", "#DC2626", "#FFFFFF"),
    StudioState.STATE_FAULT: ("FAULT", "#B91C1C", "#FFFFFF"),
    StudioState.STATE_RECOVERING: ("RECOVERING", "#7C3AED", "#FFFFFF"),
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
    GripperState.BLOCKED: ("BLOCKED", "#991B1B"),
    GripperState.STOPPED: ("STOPPED", "#64748B"),
    GripperState.OPENING: ("OPENING", "#0284C7"),
    GripperState.CLOSING: ("CLOSING", "#0284C7"),
    GripperState.OPEN: ("OPEN", "#16A34A"),
    GripperState.CLOSED: ("CLOSED", "#0F766E"),
    GripperState.GRASPED: ("GRASPED", "#059669"),
    GripperState.FAULT: ("FAULT", "#DC2626"),
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
    background-color: #F3F5F8;
    color: #0F172A;
    font-family: "Noto Sans", "Liberation Sans", sans-serif;
    font-size: 10pt;
}
QScrollArea, QWidget#MonitorPanel {
    background-color: #F3F5F8;
    color: #0F172A;
}
QFrame#HeaderBar, QFrame#ToolbarRow, QFrame#StudioCard, QFrame#CompileBanner {
    background-color: #FFFFFF;
    border: 1px solid #DDE3EC;
    border-radius: 8px;
}
QFrame#CompileBanner {
    border-radius: 6px;
}
QFrame#CardDivider, QFrame#ToolbarDivider {
    background-color: #E2E8F0;
    border: none;
}
QLabel {
    color: #1E293B;
    background-color: transparent;
}
QLabel#CardTitle {
    color: #0F172A;
    font-weight: bold;
    font-size: 10pt;
}
QLabel#FieldKey {
    color: #64748B;
    font-weight: 600;
}
QLabel#FieldVal {
    color: #0F172A;
    font-weight: 500;
}
QPlainTextEdit#EventLog {
    background-color: #FFFFFF;
    color: #1E293B;
    border: 1px solid #E2E8F0;
    border-radius: 6px;
    selection-background-color: #DBEAFE;
    selection-color: #1E3A8A;
}
QTableWidget {
    background-color: #FFFFFF;
    alternate-background-color: #F8FAFC;
    color: #0F172A;
    border: 1px solid #E2E8F0;
    border-radius: 6px;
    gridline-color: #E2E8F0;
    selection-background-color: #DBEAFE;
    selection-color: #1E3A8A;
}
QHeaderView::section {
    background-color: #F8FAFC;
    color: #334155;
    border: none;
    border-bottom: 1px solid #DDE3EC;
    border-right: 1px solid #E2E8F0;
    padding: 6px 8px;
    font-weight: bold;
}
QPushButton {
    background-color: #FFFFFF;
    color: #1E293B;
    border: 1px solid #CBD5E1;
    border-radius: 6px;
    padding: 5px 12px;
    font-weight: 600;
    min-height: 20px;
}
QPushButton:hover:enabled {
    background-color: #F8FAFC;
    border-color: #94A3B8;
}
QPushButton:pressed:enabled {
    background-color: #EFF6FF;
}
QPushButton:disabled {
    background-color: #F1F5F9;
    color: #94A3B8;
    border: 1px solid #E2E8F0;
}
QPushButton#BtnCompile:enabled {
    background-color: #2563EB;
    color: #FFFFFF;
    border-color: #1D4ED8;
}
QPushButton#BtnCompile:hover:enabled {
    background-color: #1D4ED8;
}
QPushButton#BtnRun:enabled {
    background-color: #16A34A;
    color: #FFFFFF;
    border-color: #15803D;
}
QPushButton#BtnRun:hover:enabled {
    background-color: #15803D;
}
QPushButton#BtnStop:enabled {
    background-color: #D97706;
    color: #FFFFFF;
    border-color: #B45309;
}
QPushButton#BtnStop:hover:enabled {
    background-color: #B45309;
}
QPushButton#BtnRecover:enabled {
    background-color: #7C3AED;
    color: #FFFFFF;
    border-color: #6D28D9;
}
QPushButton#BtnRecover:hover:enabled {
    background-color: #6D28D9;
}
QPushButton#BtnEstop {
    background-color: #DC2626;
    color: #FFFFFF;
    border: 2px solid #B91C1C;
    border-radius: 6px;
    font-weight: bold;
    font-size: 10pt;
    padding: 6px 16px;
}
QPushButton#BtnEstop:hover {
    background-color: #B91C1C;
}
QPushButton#BtnGripOpen:enabled {
    background-color: #0284C7;
    color: #FFFFFF;
    border-color: #0369A1;
}
QPushButton#BtnGripOpen:hover:enabled {
    background-color: #0369A1;
}
QPushButton#BtnGripClose:enabled {
    background-color: #0F766E;
    color: #FFFFFF;
    border-color: #115E59;
}
QPushButton#BtnGripClose:hover:enabled {
    background-color: #115E59;
}
QComboBox {
    background-color: #FFFFFF;
    color: #0F172A;
    border: 1px solid #CBD5E1;
    border-radius: 6px;
    padding: 4px 10px;
    min-height: 20px;
}
QComboBox QAbstractItemView {
    background-color: #FFFFFF;
    color: #0F172A;
    border: 1px solid #CBD5E1;
    selection-background-color: #DBEAFE;
    selection-color: #1E3A8A;
}
QCheckBox {
    color: #1E293B;
    font-weight: 600;
    spacing: 7px;
    background-color: transparent;
}
QProgressBar {
    background-color: #F1F5F9;
    border: 1px solid #DDE3EC;
    border-radius: 5px;
    text-align: center;
    color: #0F172A;
    font-weight: bold;
    height: 18px;
}
QProgressBar::chunk {
    background-color: #60A5FA;
    border-radius: 4px;
}
QSplitter::handle {
    background-color: #E2E8F0;
}
QSplitter::handle:horizontal {
    width: 4px;
}
QSplitter::handle:vertical {
    height: 4px;
}
"""


def _styled_pill(bg_hex, fg_hex="#FFFFFF"):
    return (
        f"background-color: {bg_hex}; color: {fg_hex}; border-radius: 5px; "
        "padding: 3px 10px; font-weight: bold; font-size: 9pt;"
    )


def _make_card(title_text):
    """Create a white border card with a crisp header and return (frame, content_layout)."""
    card = QFrame()
    card.setObjectName("StudioCard")
    vbox = QVBoxLayout(card)
    vbox.setContentsMargins(11, 7, 11, 8)
    vbox.setSpacing(5)

    lbl_title = QLabel(title_text)
    lbl_title.setObjectName("CardTitle")
    vbox.addWidget(lbl_title)

    divider = QFrame()
    divider.setObjectName("CardDivider")
    divider.setFixedHeight(1)
    vbox.addWidget(divider)

    content_layout = QVBoxLayout()
    content_layout.setContentsMargins(0, 1, 0, 0)
    content_layout.setSpacing(4)
    vbox.addLayout(content_layout)
    return card, content_layout


def _key_label(text):
    lbl = QLabel(text)
    lbl.setObjectName("FieldKey")
    return lbl


class StudioMainWindow(QMainWindow):
    """End-user industrial trajectory programming and execution Studio IDE."""

    def __init__(self, ros_worker, parent=None):
        super().__init__(parent)
        self.worker = ros_worker
        self.setWindowTitle("Robot Industrial Trajectory Studio")
        self.resize(1800, 1400)
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
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(8)

        main_layout.addWidget(self._build_header_bar())
        main_layout.addWidget(self._build_authoring_toolbar())
        main_layout.addWidget(self._build_execution_toolbar())

        # Main workspace splitter: Left IDE / Diagnostics, Right Live Telemetry & Gripper
        h_splitter = QSplitter(Qt.Horizontal)
        h_splitter.addWidget(self._build_left_ide_pane())
        h_splitter.addWidget(self._build_right_monitor_pane())
        h_splitter.setStretchFactor(0, 58)
        h_splitter.setStretchFactor(1, 42)
        h_splitter.setSizes([1050, 730])
        main_layout.addWidget(h_splitter, 1)

        # Load default industrial sample cleanly
        first_key = next(iter(PRESET_PROGRAMS))
        self.editor.setPlainText(PRESET_PROGRAMS[first_key])
        self._mark_source_dirty(False)
        self.lbl_compile_summary.setText("Not compiled - click Compile to plan & validate")
        self.lbl_compile_summary.setStyleSheet("color: #475569; font-weight: 600;")

        self._connect_signals()
        self._refresh_button_states()

    # ------------------------------------------------------------------ UI Builders

    def _build_header_bar(self):
        frame = QFrame()
        frame.setObjectName("HeaderBar")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(8)

        title = QLabel("ROBOT STUDIO")
        title.setFont(QFont(_UI_FONT_FAMILY, 12, QFont.Bold))
        title.setStyleSheet("color: #1D4ED8;")
        layout.addWidget(title)

        layout.addSpacing(4)
        self.lbl_state_badge = QLabel("BOOT INIT")
        self.lbl_state_badge.setStyleSheet(_styled_pill("#64748B"))
        layout.addWidget(self.lbl_state_badge)

        layout.addSpacing(6)
        self.lbl_joint_fresh = QLabel("JOINTS: STALE")
        self.lbl_joint_fresh.setStyleSheet(_styled_pill("#64748B"))
        layout.addWidget(self.lbl_joint_fresh)

        self.lbl_ctrl_fresh = QLabel("CTRL: STALE")
        self.lbl_ctrl_fresh.setStyleSheet(_styled_pill("#64748B"))
        layout.addWidget(self.lbl_ctrl_fresh)

        self.lbl_grip_fresh = QLabel("GRIPPER: OFFLINE")
        self.lbl_grip_fresh.setStyleSheet(_styled_pill("#64748B"))
        layout.addWidget(self.lbl_grip_fresh)

        layout.addStretch(1)

        self.btn_estop = QPushButton("EMERGENCY STOP")
        self.btn_estop.setObjectName("BtnEstop")
        self.btn_estop.setMinimumWidth(165)
        layout.addWidget(self.btn_estop)
        return frame

    def _build_authoring_toolbar(self):
        frame = QFrame()
        frame.setObjectName("ToolbarRow")
        outer = QHBoxLayout(frame)
        outer.setContentsMargins(10, 6, 10, 6)
        outer.setSpacing(8)

        self.btn_new = QPushButton("New")
        self.btn_open = QPushButton("Open")
        self.btn_save = QPushButton("Save")
        outer.addWidget(self.btn_new)
        outer.addWidget(self.btn_open)
        outer.addWidget(self.btn_save)

        sep1 = QFrame()
        sep1.setObjectName("ToolbarDivider")
        sep1.setFixedWidth(1)
        outer.addWidget(sep1)

        self.cmb_presets = QComboBox()
        for name in PRESET_PROGRAMS:
            self.cmb_presets.addItem(name)
        self.cmb_presets.setMinimumWidth(285)
        outer.addWidget(self.cmb_presets)

        self.btn_load_preset = QPushButton("Load Example")
        outer.addWidget(self.btn_load_preset)

        sep2 = QFrame()
        sep2.setObjectName("ToolbarDivider")
        sep2.setFixedWidth(1)
        outer.addWidget(sep2)

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
        return frame

    def _build_execution_toolbar(self):
        frame = QFrame()
        frame.setObjectName("ToolbarRow")
        outer = QHBoxLayout(frame)
        outer.setContentsMargins(10, 6, 10, 6)
        outer.setSpacing(8)

        self.btn_compile = QPushButton("Compile")
        self.btn_compile.setObjectName("BtnCompile")
        self.btn_compile.setMinimumWidth(96)

        self.btn_run_resume = QPushButton("Run")
        self.btn_run_resume.setObjectName("BtnRun")
        self.btn_run_resume.setMinimumWidth(88)

        self.btn_pause = QPushButton("Pause")
        self.btn_step = QPushButton("Step")

        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setObjectName("BtnStop")

        self.btn_reset_fault = QPushButton("Reset Fault")

        self.btn_recover = QPushButton("Recover E-STOP")
        self.btn_recover.setObjectName("BtnRecover")

        self.btn_clear = QPushButton("Clear")

        outer.addWidget(self.btn_compile)

        sep1 = QFrame()
        sep1.setObjectName("ToolbarDivider")
        sep1.setFixedWidth(1)
        outer.addWidget(sep1)

        for btn in (self.btn_run_resume, self.btn_pause, self.btn_step, self.btn_stop):
            outer.addWidget(btn)

        sep2 = QFrame()
        sep2.setObjectName("ToolbarDivider")
        sep2.setFixedWidth(1)
        outer.addWidget(sep2)

        for btn in (self.btn_reset_fault, self.btn_recover, self.btn_clear):
            outer.addWidget(btn)

        outer.addStretch(1)
        return frame

    def _build_left_ide_pane(self):
        v_splitter = QSplitter(Qt.Vertical)

        # Top: Editor with compile summary strip
        editor_container = QWidget()
        e_layout = QVBoxLayout(editor_container)
        e_layout.setContentsMargins(0, 0, 0, 0)
        e_layout.setSpacing(6)

        banner = QFrame()
        banner.setObjectName("CompileBanner")
        b_layout = QHBoxLayout(banner)
        b_layout.setContentsMargins(10, 6, 10, 6)
        self.lbl_compile_summary = QLabel("Ready")
        self.lbl_compile_summary.setStyleSheet("color: #475569; font-weight: 600;")
        self.lbl_dirty_flag = QLabel("")
        self.lbl_dirty_flag.setStyleSheet("color: #B45309; font-weight: bold;")
        b_layout.addWidget(self.lbl_compile_summary, 1)
        b_layout.addWidget(self.lbl_dirty_flag)
        e_layout.addWidget(banner)

        self.editor = CodeEditor()
        e_layout.addWidget(self.editor, 1)
        v_splitter.addWidget(editor_container)

        # Bottom: Diagnostics card
        diag_card, d_layout = _make_card("Compiler Diagnostics (click row to jump to line)")
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

        v_splitter.addWidget(diag_card)
        v_splitter.setStretchFactor(0, 72)
        v_splitter.setStretchFactor(1, 28)
        return v_splitter

    def _build_right_monitor_pane(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(470)

        panel = QWidget()
        panel.setObjectName("MonitorPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(4, 0, 2, 0)
        layout.setSpacing(6)

        # 1. Execution progress card
        prog_card, prog_v = _make_card("Program Execution & Progress")
        p_grid = QGridLayout()
        p_grid.setHorizontalSpacing(10)
        p_grid.setVerticalSpacing(4)
        p_grid.setColumnStretch(1, 1)
        p_grid.setColumnStretch(3, 1)

        self.lbl_prog_id = QLabel("--")
        self.lbl_prog_id.setFont(QFont(_MONO_FONT_FAMILY, 9))
        self.lbl_prog_id.setWordWrap(True)
        self.lbl_block_idx = QLabel("0 / 0")
        self.lbl_active_line = QLabel("--")
        self.lbl_exec_detail = QLabel("--")
        self.lbl_exec_detail.setWordWrap(True)

        self.bar_block_progress = QProgressBar()
        self.bar_block_progress.setRange(0, 100)
        self.bar_total_progress = QProgressBar()
        self.bar_total_progress.setRange(0, 100)

        p_grid.addWidget(_key_label("Program ID:"), 0, 0)
        p_grid.addWidget(self.lbl_prog_id, 0, 1, 1, 3)
        p_grid.addWidget(_key_label("Block:"), 1, 0)
        p_grid.addWidget(self.lbl_block_idx, 1, 1)
        p_grid.addWidget(_key_label("Active Line:"), 1, 2)
        p_grid.addWidget(self.lbl_active_line, 1, 3)
        p_grid.addWidget(_key_label("Block %:"), 2, 0)
        p_grid.addWidget(self.bar_block_progress, 2, 1, 1, 3)
        p_grid.addWidget(_key_label("Total %:"), 3, 0)
        p_grid.addWidget(self.bar_total_progress, 3, 1, 1, 3)
        p_grid.addWidget(_key_label("Status:"), 4, 0, Qt.AlignTop)
        p_grid.addWidget(self.lbl_exec_detail, 4, 1, 1, 3)
        prog_v.addLayout(p_grid)
        layout.addWidget(prog_card)

        # 2. Controller & Impedance Telemetry Card
        ctrl_card, ctrl_v = _make_card("Controller & Impedance Tracking")
        c_grid = QGridLayout()
        c_grid.setHorizontalSpacing(10)
        c_grid.setVerticalSpacing(6)
        c_grid.setColumnStretch(1, 1)

        self.lbl_ctrl_mode = QLabel("UNKNOWN")
        self.lbl_ctrl_mode.setStyleSheet("color: #1D4ED8; font-weight: bold;")
        self.lbl_ctrl_mode.setWordWrap(True)
        self.lbl_estop_reason = QLabel("None")
        self.lbl_estop_reason.setWordWrap(True)
        self.lbl_pos_err = QLabel("0.00 mm  [0.0, 0.0, 0.0]")
        self.lbl_pos_err.setFont(QFont(_MONO_FONT_FAMILY, 9))
        self.lbl_pos_err.setWordWrap(True)
        self.lbl_rot_err = QLabel("0.00 deg [0.0, 0.0, 0.0]")
        self.lbl_rot_err.setFont(QFont(_MONO_FONT_FAMILY, 9))
        self.lbl_rot_err.setWordWrap(True)

        c_grid.addWidget(_key_label("CTC Mode:"), 0, 0)
        c_grid.addWidget(self.lbl_ctrl_mode, 0, 1)
        c_grid.addWidget(_key_label("E-STOP / Fault:"), 1, 0, Qt.AlignTop)
        c_grid.addWidget(self.lbl_estop_reason, 1, 1)
        c_grid.addWidget(_key_label("Impedance Pos Err:"), 2, 0)
        c_grid.addWidget(self.lbl_pos_err, 2, 1)
        c_grid.addWidget(_key_label("Impedance Rot Err:"), 3, 0)
        c_grid.addWidget(self.lbl_rot_err, 3, 1)
        ctrl_v.addLayout(c_grid)
        layout.addWidget(ctrl_card)

        # 3. Live Robot Joint & TCP Telemetry Card
        rob_card, r_layout = _make_card("Live Robot Feedback")

        self.lbl_tcp_pose = QLabel("TCP: awaiting TF link_base -> link_eef...")
        self.lbl_tcp_pose.setFont(QFont(_MONO_FONT_FAMILY, 9))
        self.lbl_tcp_pose.setStyleSheet("color: #0369A1; font-weight: 600;")
        self.lbl_tcp_pose.setWordWrap(True)
        r_layout.addWidget(self.lbl_tcp_pose)

        j_grid = QGridLayout()
        j_grid.setHorizontalSpacing(8)
        j_grid.setVerticalSpacing(4)
        j_grid.setColumnStretch(1, 1)
        j_grid.setColumnStretch(3, 1)
        self._joint_labels = []
        mono_joint_font = QFont(_MONO_FONT_FAMILY, 9)
        for idx in range(6):
            row = idx // 2
            col_base = (idx % 2) * 2
            name_lbl = _key_label(f"J{idx + 1}:")
            val_lbl = QLabel("+0.000 rad (+0.0 deg)")
            val_lbl.setFont(mono_joint_font)
            j_grid.addWidget(name_lbl, row, col_base)
            j_grid.addWidget(val_lbl, row, col_base + 1)
            self._joint_labels.append(val_lbl)
        r_layout.addLayout(j_grid)
        layout.addWidget(rob_card)

        # 4. Standalone Decoupled Gripper Card
        grip_card, g_layout = _make_card("End-Effector (Decoupled Gripper Control)")

        g_info = QHBoxLayout()
        g_info.setSpacing(10)
        self.lbl_gripper_state = QLabel("OFFLINE")
        self.lbl_gripper_state.setStyleSheet(_styled_pill("#64748B"))
        self.lbl_gripper_metrics = QLabel("Opening: -- mm | Effort: -- N")
        self.lbl_gripper_metrics.setFont(QFont(_MONO_FONT_FAMILY, 9))
        g_info.addWidget(self.lbl_gripper_state)
        g_info.addWidget(self.lbl_gripper_metrics, 1)
        g_layout.addLayout(g_info)

        g_btns = QHBoxLayout()
        g_btns.setSpacing(8)
        self.btn_grip_open = QPushButton("Open Gripper")
        self.btn_grip_open.setObjectName("BtnGripOpen")
        self.btn_grip_open.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_grip_close = QPushButton("Close Gripper")
        self.btn_grip_close.setObjectName("BtnGripClose")
        self.btn_grip_close.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_grip_stop = QPushButton("Stop")
        self.btn_grip_stop.setToolTip("Stop gripper actuation")
        self.btn_grip_stop.setMinimumWidth(78)
        g_btns.addWidget(self.btn_grip_open, 1)
        g_btns.addWidget(self.btn_grip_close, 1)
        g_btns.addWidget(self.btn_grip_stop, 0)
        g_layout.addLayout(g_btns)
        layout.addWidget(grip_card)

        # 5. Event Console Log Card
        log_card, l_layout = _make_card("Studio Event Log")
        self.txt_log = QPlainTextEdit()
        self.txt_log.setObjectName("EventLog")
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(300)
        self.txt_log.setMinimumHeight(95)
        self.txt_log.setFont(QFont(_MONO_FONT_FAMILY, 9))
        l_layout.addWidget(self.txt_log)
        layout.addWidget(log_card, 1)

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
        self.editor.setPlainText("# New RobotScript program\nMoveJ([0.0, 0.35, 1.0, 0.0, 0.65, 0.0], vel=0.4, acc=0.4)\n")
        self._current_file_path = None
        self.editor.set_diagnostic_markers([], [])
        self.diag_table.setRowCount(0)
        self._mark_source_dirty(True)

    def _on_open_script(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open RobotScript Program", "", "RobotScript (*.l6 *.txt);;All Files (*)"
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
                self, "Save RobotScript Program", "program.l6", "RobotScript (*.l6);;All Files (*)"
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
        self.lbl_compile_summary.setStyleSheet("color: #1D4ED8; font-weight: 600;")
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
            sev_item.setForeground(QColor("#DC2626" if is_err else "#B45309"))
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
            self.lbl_compile_summary.setStyleSheet("color: #15803D; font-weight: 600;")
            self.append_log(summary)
        else:
            err_count = sum(1 for d in self._diagnostics if d.severity == CompileDiagnostic.SEVERITY_ERROR)
            summary = f"Compile FAILED ({err_count} error(s)) - click diagnostic row to locate"
            self.lbl_compile_summary.setText(summary)
            self.lbl_compile_summary.setStyleSheet("color: #DC2626; font-weight: 600;")
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

        name, bg, fg = _STATE_META.get(msg.state, (f"STATE_{msg.state}", "#64748B", "#FFFFFF"))
        self.lbl_state_badge.setText(name)
        self.lbl_state_badge.setStyleSheet(_styled_pill(bg, fg))

        self.lbl_joint_fresh.setText(
            "JOINTS: FRESH" if msg.joint_states_fresh else "JOINTS: STALE"
        )
        self.lbl_joint_fresh.setStyleSheet(
            _styled_pill("#16A34A" if msg.joint_states_fresh else "#DC2626")
        )
        self.lbl_ctrl_fresh.setText("CTRL: FRESH" if msg.controller_fresh else "CTRL: STALE")
        self.lbl_ctrl_fresh.setStyleSheet(
            _styled_pill("#16A34A" if msg.controller_fresh else "#DC2626")
        )
        self.lbl_grip_fresh.setText("GRIPPER: READY" if msg.gripper_ready else "GRIPPER: NOT READY")
        self.lbl_grip_fresh.setStyleSheet(
            _styled_pill("#0F766E" if msg.gripper_ready else "#64748B")
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
            self.lbl_estop_reason.setStyleSheet("color: #DC2626; font-weight: bold;")
        elif msg.last_error_code != 0:
            self.lbl_estop_reason.setText(f"Fault code {msg.last_error_code}")
            self.lbl_estop_reason.setStyleSheet("color: #B45309; font-weight: bold;")
        else:
            self.lbl_estop_reason.setText("None")
            self.lbl_estop_reason.setStyleSheet("color: #64748B;")

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
            self.lbl_compile_summary.setStyleSheet("color: #475569; font-weight: 600;")

        self._refresh_button_states()

    def on_gripper_state(self, msg):
        gname, gbg = _GRIPPER_STATE_NAMES.get(msg.state, (f"STATE_{msg.state}", "#64748B"))
        self.lbl_gripper_state.setText(gname)
        self.lbl_gripper_state.setStyleSheet(_styled_pill(gbg))
        pos_mm = (msg.position[0] - msg.position[1]) * 1000.0
        self.lbl_gripper_metrics.setText(f"Opening: {pos_mm:5.1f} mm | Effort: {msg.effort:5.1f} N")

    def on_telemetry(self, q_list, _dq_list, tcp6):
        if q_list is not None:
            self._latest_q = q_list
            for idx, val in enumerate(q_list):
                self._joint_labels[idx].setText(f"{val:+6.3f} rad ({math.degrees(val):+5.1f} deg)")
        if tcp6 is not None:
            self._latest_tcp = tcp6
            x, y, z, r, p, w = tcp6
            self.lbl_tcp_pose.setText(
                f"XYZ: [{x:6.4f}, {y:6.4f}, {z:6.4f}] m\n"
                f"RPY: [{r:6.3f}, {p:6.3f}, {w:6.3f}] rad"
            )
