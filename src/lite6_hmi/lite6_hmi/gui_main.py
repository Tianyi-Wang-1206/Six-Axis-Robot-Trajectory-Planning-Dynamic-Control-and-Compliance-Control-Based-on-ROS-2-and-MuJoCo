#!/usr/bin/env python3
import os
import signal
import sys
import math
from lite6_motion_contract import MAX_PAYLOAD_MASS_KG, MAX_PAYLOAD_COM_RADIUS_M
import numpy as np
import rclpy
from lite6_interfaces.action import IndustrialMotion
from lite6_interfaces.srv import GripperCommand
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QLineEdit, QTabWidget, QGroupBox,
    QGridLayout, QMessageBox, QFrame, QStyle, QProgressBar
)
from PyQt5.QtGui import QFont, QDoubleValidator, QPalette, QColor
from lite6_hmi.supervisor_worker import ROS2Worker, Lite6StateMachine
from lite6_hmi.compliance_tab import ComplianceTab


class PyQtHMI(QMainWindow):
    """
    Main Thread PyQt5 HMI Window.
    Processes telemetry and state transitions strictly via Qt Slots.
    """
    def __init__(self, worker):
        super().__init__()
        self.worker = worker
        self.setWindowTitle("Robot - Industrial HMI Interface")
        self.resize(920, 780)

        # Style definition via QSS (Modern Slate Industrial Theme)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f4f6f9;
            }
            QGroupBox {
                font-size: 11pt;
                font-weight: bold;
                border: 2px solid #bdc3c7;
                border-radius: 6px;
                margin-top: 12px;
                background-color: #ffffff;
                padding-top: 18px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 5px;
                color: #2c3e50;
            }
            QLabel {
                font-family: "Segoe UI", "Ubuntu";
                font-size: 10pt;
                color: #34495e;
            }
            QLineEdit {
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                padding: 5px;
                font-family: "Consolas", "Ubuntu Mono";
                font-size: 10pt;
                background-color: #ffffff;
            }
            QLineEdit:focus {
                border: 1px solid #3498db;
            }
            QLineEdit:disabled {
                background-color: #ecf0f1;
                color: #7f8c8d;
            }
            QSlider::groove:horizontal {
                border: 1px solid #bbb;
                background: #e0e0e0;
                height: 10px;
                border-radius: 5px;
            }
            QSlider::sub-page:horizontal {
                background: #3498db;
                border-radius: 5px;
            }
            QSlider::handle:horizontal {
                background: #2c3e50;
                border: 1px solid #1a252f;
                width: 20px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 10px;
            }
            QTabWidget::pane {
                border: 1px solid #bdc3c7;
                background-color: #f4f6f9;
            }
            QTabBar::tab {
                background: #e2e6ea;
                border: 1px solid #bdc3c7;
                padding: 8px 12px;
                font-family: "Segoe UI", "Ubuntu";
                font-weight: bold;
                font-size: 7pt;
                color: #2c3e50;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                border-bottom-color: #ffffff;
            }
            QPushButton {
                background-color: #34495e;
                color: #ffffff;
                font-family: "Segoe UI", "Ubuntu";
                font-size: 10pt;
                font-weight: bold;
                border: none;
                border-radius: 5px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #415b76;
            }
            QPushButton:pressed {
                background-color: #2c3e50;
            }
        """)

        # Shared limits
        self.j_limits = [
            [-360.0, 360.0], [-150.0, 150.0], [0.0, 285.0], 
            [-360.0, 360.0], [-93.0, 93.0], [-360.0, 360.0]
        ]
        self.exact_q_target = [0.0] * 6
        self.exact_p_target = [0.0] * 6
        self.exact_aux_target = [0.0] * 6

        # Shared double validator
        self.float_validator = QDoubleValidator(-10000.0, 10000.0, 4, self)
        self.float_validator.setNotation(QDoubleValidator.StandardNotation)

        # Build HMI Layout
        self.setup_ui()
        self._command_notice = None
        self._estop_notice = None

        # Connect ROS worker signals to GUI Main Thread slots
        self.worker.telemetry_signal.connect(self.on_telemetry_updated)
        self.worker.state_signal.connect(self.on_state_updated)
        self.worker.sysid_yaml_signal.connect(self.on_sysid_yaml_updated)
        self.worker.sysid_preparation_signal.connect(self.on_sysid_preparation_updated)
        # A GUI command can report rejection while holding the worker's state
        # lock. Always deliver the notice after that command has unwound.
        self.worker.command_error_signal.connect(self.on_command_error, Qt.QueuedConnection)
        self.worker.estop_fault_signal.connect(self.on_estop_fault, Qt.QueuedConnection)
        self.worker.recovery_complete_signal.connect(self.on_recovery_completed)
        self.worker.gripper_signal.connect(self.on_gripper_updated)
        self.worker.motion_feedback_signal.connect(self.on_motion_feedback)
        self._supervisor_offline_displayed = False

        # Render the cached snapshot as well as future transitions. A Worker
        # supplied by another caller may already have received its first state.
        with self.worker.state_lock:
            state, detail = self.worker.state, self.worker.error_msg
        self.on_state_updated(state, detail)

        # Connection timeout watchdog timer
        self.watchdog_timer = QTimer()
        self.watchdog_timer.timeout.connect(self.check_connection_watchdog)
        self.watchdog_timer.start(500)
        self.gripper_timer = QTimer(self)
        self.gripper_timer.timeout.connect(self.refresh_gripper_controls)
        self.gripper_timer.start(100)

    def setup_gripper_tab(self, tab):
        layout = QVBoxLayout(tab)
        self.gripper_status = QLabel("Waiting for gripper feedback...")
        self.gripper_status.setWordWrap(True)
        layout.addWidget(self.gripper_status)
        note = QLabel("Wide gripper · Independent Open / Close / Stop\n"
                      "Stop turns off the motor drive; it does not lock the fingers.\n"
                      "Grasp feedback is an estimate, not a guarantee of a secure hold.\n"
                      "TCP: 81.1 mm along flange Z.")
        note.setWordWrap(True)
        layout.addWidget(note)
        actions = QHBoxLayout()
        self.btn_gripper_open = QPushButton("Open")
        self.btn_gripper_close = QPushButton("Close")
        self.btn_gripper_stop = QPushButton("Stop")
        self.btn_gripper_open.clicked.connect(
            lambda: self.worker.command_gripper(GripperCommand.Request.OPEN))
        self.btn_gripper_close.clicked.connect(
            lambda: self.worker.command_gripper(GripperCommand.Request.CLOSE))
        self.btn_gripper_stop.clicked.connect(
            lambda: self.worker.command_gripper(GripperCommand.Request.STOP))
        for button in (self.btn_gripper_open, self.btn_gripper_close, self.btn_gripper_stop):
            actions.addWidget(button)
            button.setEnabled(False)
        layout.addLayout(actions)
        layout.addStretch()

    def on_gripper_updated(self, msg):
        self.gripper_status.setText(f"{msg.detail}\nMotor effort: {msg.effort:.2f} N")
        self.refresh_gripper_controls()

    def refresh_gripper_controls(self):
        enabled = self.worker.gripper_ready()
        for button in (self.btn_gripper_open, self.btn_gripper_close, self.btn_gripper_stop):
            button.setEnabled(enabled)
        #The transport checks heartbeat age independently of the state label.
        import time
        with self.worker.state_lock:
            stale = time.monotonic() - self.worker.last_gripper_rx_time > 0.25
        if stale:
            self.gripper_status.setText("Gripper feedback offline; controls disabled")
        self.refresh_payload_status()

    def setup_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        master_layout = QVBoxLayout(main_widget)
        master_layout.setContentsMargins(15, 15, 15, 15)
        master_layout.setSpacing(10)

        # 1. State Machine Banner Frame
        self.state_banner = QLabel("INITIALIZING SYSTEM...")
        self.state_banner.setAlignment(Qt.AlignCenter)
        self.state_banner.setFont(QFont("Segoe UI", 16, QFont.Bold))
        self.state_banner.setFixedHeight(60)
        self.set_banner_color("#2c3e50") # Deep charcoal default
        master_layout.addWidget(self.state_banner)

        # 2. Main Middle Workspace split: Left Telemetry, Right Controls
        workspace_layout = QHBoxLayout()
        master_layout.addLayout(workspace_layout)

        # Col 1: Read-Only Realtime Telemetry
        telemetry_container = QWidget()
        telemetry_container.setFixedWidth(260)
        col1_layout = QVBoxLayout(telemetry_container)
        col1_layout.setContentsMargins(0, 0, 0, 0)
        
        tele_group = QGroupBox("Realtime Telemetry")
        col1_layout.addWidget(tele_group)
        tele_layout = QVBoxLayout(tele_group)
        tele_layout.setSpacing(10)

        # Joint positions
        tele_layout.addWidget(QLabel("Joint Angles (Physical Position)", font=QFont("Segoe UI", 10, QFont.Bold)))
        self.lbl_q_curr = []
        for i in range(6):
            lbl = QLabel(f" J{i+1}:   0.00 °")
            lbl.setFont(QFont("Consolas", 10))
            tele_layout.addWidget(lbl)
            self.lbl_q_curr.append(lbl)

        # Horizontal Divider Line
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        tele_layout.addWidget(line)

        # TCP Pose
        tele_layout.addWidget(QLabel("TCP Frame Pose (Base Link)", font=QFont("Segoe UI", 10, QFont.Bold)))
        self.lbl_p_curr = []
        labels = ['X (mm)', 'Y (mm)', 'Z (mm)', 'Roll (°)', 'Pitch (°)', 'Yaw (°)']
        for name in labels:
            lbl = QLabel(f" {name:9s}:   0.00")
            lbl.setFont(QFont("Consolas", 10))
            tele_layout.addWidget(lbl)
            self.lbl_p_curr.append(lbl)

        line_m = QFrame()
        line_m.setFrameShape(QFrame.HLine)
        line_m.setFrameShadow(QFrame.Sunken)
        tele_layout.addWidget(line_m)

        self.lbl_motion_feedback = QLabel("Motion: IDLE")
        self.lbl_motion_feedback.setFont(QFont("Segoe UI", 9, QFont.Bold))
        tele_layout.addWidget(self.lbl_motion_feedback)
        self.bar_motion_progress = QProgressBar()
        self.bar_motion_progress.setRange(0, 100)
        self.bar_motion_progress.setValue(0)
        self.bar_motion_progress.setFixedHeight(16)
        tele_layout.addWidget(self.bar_motion_progress)

        tele_layout.addStretch()
        workspace_layout.addWidget(telemetry_container)

        # Col 2: Action tabs
        self.tabs = QTabWidget()
        workspace_layout.addWidget(self.tabs)

        # Tab A: Manual Control
        tab_manual = QWidget()
        self.tabs.addTab(tab_manual, "Manual Control")
        self.setup_manual_tab(tab_manual)

        # Tab B: Friction Identification
        tab_sysid = QWidget()
        self.tabs.addTab(tab_sysid, "Friction Identification")
        self.setup_sysid_tab(tab_sysid)

        tab_gripper = QWidget()
        self.tabs.addTab(tab_gripper, "Gripper")
        self.setup_gripper_tab(tab_gripper)

        self.compliance_tab = ComplianceTab(self.worker)
        self.tabs.addTab(self.compliance_tab, "Compliance / Teaching")

        # 3. Bottom Control Frame: Overrides, Home, Fault Recovery and E-Stop
        bot_frame = QWidget()
        bot_frame.setStyleSheet("background-color: #dee2e6; border-radius: 4px;")
        bot_frame.setFixedHeight(95)
        master_layout.addWidget(bot_frame)
        
        bot_layout = QHBoxLayout(bot_frame)
        bot_layout.setContentsMargins(15, 10, 15, 10)

        # Override adjustments
        overrides_container = QWidget()
        over_layout = QGridLayout(overrides_container)
        over_layout.setContentsMargins(0, 0, 0, 0)
        over_layout.setSpacing(5)

        # Velocity Override
        lbl_v = QLabel("Velocity Scale:")
        lbl_v.setStyleSheet("font-weight: bold; background-color: transparent;")
        over_layout.addWidget(lbl_v, 0, 0)
        
        self.v_slider = QSlider(Qt.Horizontal)
        self.v_slider.setRange(1, 100)
        self.v_slider.setValue(100)
        self.v_slider.setFixedWidth(150)
        self.v_slider.setStyleSheet("background-color: transparent;")
        over_layout.addWidget(self.v_slider, 0, 1)

        self.v_entry = QLineEdit("1.00")
        self.v_entry.setFixedWidth(55)
        self.v_entry.setAlignment(Qt.AlignCenter)
        self.v_entry.setValidator(self.float_validator)
        over_layout.addWidget(self.v_entry, 0, 2)

        # Acceleration Override
        lbl_a = QLabel("Acceleration Scale:")
        lbl_a.setStyleSheet("font-weight: bold; background-color: transparent;")
        over_layout.addWidget(lbl_a, 1, 0)

        self.a_slider = QSlider(Qt.Horizontal)
        self.a_slider.setRange(1, 100)
        self.a_slider.setValue(100)
        self.a_slider.setFixedWidth(150)
        self.a_slider.setStyleSheet("background-color: transparent;")
        over_layout.addWidget(self.a_slider, 1, 1)

        self.a_entry = QLineEdit("1.00")
        self.a_entry.setFixedWidth(55)
        self.a_entry.setAlignment(Qt.AlignCenter)
        self.a_entry.setValidator(self.float_validator)
        over_layout.addWidget(self.a_entry, 1, 2)

        bot_layout.addWidget(overrides_container)
        bot_layout.addStretch()

        # Connect Slider and LineEdit for scaling metrics
        self.v_slider.valueChanged.connect(self.sync_v_slider_to_entry)
        self.v_entry.editingFinished.connect(self.sync_v_entry_to_slider)
        self.a_slider.valueChanged.connect(self.sync_a_slider_to_entry)
        self.a_entry.editingFinished.connect(self.sync_a_entry_to_slider)

        # Utility Buttons
        self.btn_home = QPushButton("⌂ HOME")
        self.btn_home.setFixedSize(110, 45)
        self.btn_home.setStyleSheet("background-color: #2c3e50; font-size: 13px; color: white;")
        self.btn_home.clicked.connect(self.cmd_home)
        bot_layout.addWidget(self.btn_home)

        self.btn_reset_fault = QPushButton("⚠ RESET FAULT")
        self.btn_reset_fault.setFixedSize(130, 45)
        self.btn_reset_fault.setStyleSheet("background-color: #f39c12; font-size: 13px; color: white;")
        self.btn_reset_fault.clicked.connect(self.gui_reset_fault)
        bot_layout.addWidget(self.btn_reset_fault)

        self.btn_estop = QPushButton("STOP (E-STOP)")
        self.btn_estop.setFixedSize(180, 45)
        self.btn_estop.setStyleSheet("background-color: #d9534f; font-size: 15px; font-weight: bold; color: white;")
        self.btn_estop.clicked.connect(lambda: self.worker.trigger_estop("SOFTWARE E-STOP PRESSED"))
        bot_layout.addWidget(self.btn_estop)

    def setup_manual_tab(self, parent):
        outer_layout = QVBoxLayout(parent)
        outer_layout.setContentsMargins(8, 8, 8, 8)
        outer_layout.setSpacing(8)

        tab_layout = QHBoxLayout()
        tab_layout.setContentsMargins(0, 0, 0, 0)
        tab_layout.setSpacing(10)
        outer_layout.addLayout(tab_layout, 1)

        # Column 1: Joint Space Control
        j_group = QGroupBox("Joint Space Commands (PTP)")
        tab_layout.addWidget(j_group)
        j_layout = QVBoxLayout(j_group)
        j_layout.setContentsMargins(15, 15, 15, 15)

        self.btn_sync_j = QPushButton("⟲ Sync target with raw state")
        self.btn_sync_j.setStyleSheet("background-color: #ecf0f1; color: #2c3e50; border: 1px solid #bdc3c7;")
        self.btn_sync_j.clicked.connect(self.sync_joints)
        j_layout.addWidget(self.btn_sync_j)

        self.j_sliders = []
        self.j_entries = []

        # Create 6 Joint controls
        for i in range(6):
            row_widget = QWidget()
            row_layout = QHBoxLayout(row_widget)
            row_layout.setContentsMargins(0, 0, 0, 0)
            
            lbl_name = QLabel(f"J{i+1}")
            lbl_name.setFont(QFont("Segoe UI", 9, QFont.Bold))
            lbl_name.setFixedWidth(20)
            row_layout.addWidget(lbl_name)

            lim = self.j_limits[i]
            lbl_low = QLabel(f"{lim[0]:.0f}°")
            lbl_low.setStyleSheet("color: gray;")
            lbl_low.setFixedWidth(35)
            lbl_low.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row_layout.addWidget(lbl_low)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(lim[0] * 100), int(lim[1] * 100))
            slider.setValue(0)
            row_layout.addWidget(slider)
            self.j_sliders.append(slider)

            entry = QLineEdit("0.00")
            entry.setFixedWidth(65)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            row_layout.addWidget(entry)
            self.j_entries.append(entry)

            lbl_high = QLabel(f"{lim[1]:.0f}°")
            lbl_high.setStyleSheet("color: gray;")
            lbl_high.setFixedWidth(35)
            row_layout.addWidget(lbl_high)

            j_layout.addWidget(row_widget)

            # Connect slider and entry dynamically
            self.connect_joint_pair(slider, entry, i)

        self.btn_move_j = QPushButton("Execute Joint Trajectory")
        self.btn_move_j.setStyleSheet("background-color: #2980b9; height: 35px; font-size: 13px;")
        self.btn_move_j.clicked.connect(self.cmd_move_j)
        j_layout.addWidget(self.btn_move_j)

        # Column 2: Cartesian Space Control
        c_group = QGroupBox("Cartesian Space Commands")
        tab_layout.addWidget(c_group)
        c_layout = QVBoxLayout(c_group)
        c_layout.setContentsMargins(15, 15, 15, 15)

        # Pose Coordinates Grid
        pose_widget = QWidget()
        pose_grid = QGridLayout(pose_widget)
        pose_grid.setContentsMargins(0, 0, 0, 0)
        pose_grid.setSpacing(8)

        lbl_target_title = QLabel("Target TCP Coordinate")
        lbl_target_title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        pose_grid.addWidget(lbl_target_title, 0, 0, 1, 2)

        self.btn_sync_p = QPushButton("⟲ Sync")
        self.btn_sync_p.setStyleSheet("background-color: #ecf0f1; color: #2c3e50; border: 1px solid #bdc3c7;")
        self.btn_sync_p.setFixedWidth(70)
        self.btn_sync_p.clicked.connect(self.sync_pose)
        pose_grid.addWidget(self.btn_sync_p, 0, 3, 1, 1, Qt.AlignRight)

        self.p_entries = []
        labels = ['X', 'Roll', 'Y', 'Pitch', 'Z', 'Yaw']
        idx_map = [0, 3, 1, 4, 2, 5]
        
        for i, idx in enumerate(idx_map):
            lbl_ax = QLabel(labels[i])
            lbl_ax.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            pose_grid.addWidget(lbl_ax, (i // 2) + 1, (i % 2) * 2)
            
            entry = QLineEdit("0.00")
            entry.setFixedWidth(80)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            pose_grid.addWidget(entry, (i // 2) + 1, (i % 2) * 2 + 1)
            self.p_entries.append(entry)

        # Position placeholders
        sorted_entries = [None] * 6
        for i, idx in enumerate(idx_map):
            sorted_entries[idx] = self.p_entries[i]
        self.p_entries = sorted_entries

        c_layout.addWidget(pose_widget)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        c_layout.addWidget(line)

        # Circular Auxiliary Frame Grid
        aux_widget = QWidget()
        aux_grid = QGridLayout(aux_widget)
        aux_grid.setContentsMargins(0, 0, 0, 0)
        aux_grid.setSpacing(8)

        lbl_aux_title = QLabel("Circular Aux Frame")
        lbl_aux_title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        aux_grid.addWidget(lbl_aux_title, 0, 0, 1, 2)

        self.btn_sync_aux = QPushButton("⟲ Sync")
        self.btn_sync_aux.setStyleSheet("background-color: #ecf0f1; color: #2c3e50; border: 1px solid #bdc3c7;")
        self.btn_sync_aux.setFixedWidth(70)
        self.btn_sync_aux.clicked.connect(self.sync_aux)
        aux_grid.addWidget(self.btn_sync_aux, 0, 3, 1, 1, Qt.AlignRight)

        self.aux_p_entries = []
        for i, idx in enumerate(idx_map):
            lbl_ax = QLabel(labels[i])
            lbl_ax.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            aux_grid.addWidget(lbl_ax, (i // 2) + 1, (i % 2) * 2)
            
            entry = QLineEdit("0.00")
            entry.setFixedWidth(80)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            entry.setStyleSheet("background-color: #e8f4f8;")
            aux_grid.addWidget(entry, (i // 2) + 1, (i % 2) * 2 + 1)
            self.aux_p_entries.append(entry)

        sorted_aux_entries = [None] * 6
        for i, idx in enumerate(idx_map):
            sorted_aux_entries[idx] = self.aux_p_entries[i]
        self.aux_p_entries = sorted_aux_entries

        c_layout.addWidget(aux_widget)
        c_layout.addStretch()

        # Action Buttons
        self.btn_move_p = QPushButton("Execute Cartesian PTP (MoveP)")
        self.btn_move_p.setStyleSheet("background-color: #5bc0de; height: 30px;")
        self.btn_move_p.clicked.connect(self.cmd_move_p)
        c_layout.addWidget(self.btn_move_p)

        self.btn_move_l = QPushButton("Execute Linear Interpolation (MoveL)")
        self.btn_move_l.setStyleSheet("background-color: #27ae60; height: 30px;")
        self.btn_move_l.clicked.connect(self.cmd_move_l)
        c_layout.addWidget(self.btn_move_l)

        self.btn_move_c = QPushButton("Execute Circular Interpolation (MoveC)")
        self.btn_move_c.setStyleSheet("background-color: #8e44ad; height: 30px;")
        self.btn_move_c.clicked.connect(self.cmd_move_c)
        c_layout.addWidget(self.btn_move_c)

        # Full-width Payload Mass & Center of Mass (TCP Frame) Configuration
        p_group = QGroupBox("Additional Payload (0-600 g) && Center of Mass (TCP Frame)")
        outer_layout.addWidget(p_group)
        p_outer = QVBoxLayout(p_group)
        p_outer.setContentsMargins(12, 8, 12, 8)
        p_outer.setSpacing(6)

        p_inputs = QHBoxLayout()
        p_inputs.setSpacing(6)

        p_inputs.addWidget(QLabel("Mass (kg):"))
        self.payload_mass_entry = QLineEdit("0.000")
        self.payload_mass_entry.setFixedWidth(64)
        self.payload_mass_entry.setAlignment(Qt.AlignCenter)
        self.payload_mass_entry.setValidator(self.float_validator)
        p_inputs.addWidget(self.payload_mass_entry)

        p_inputs.addWidget(QLabel("CoM X (mm):"))
        self.payload_com_x_entry = QLineEdit("0.0")
        self.payload_com_x_entry.setFixedWidth(56)
        self.payload_com_x_entry.setAlignment(Qt.AlignCenter)
        self.payload_com_x_entry.setValidator(self.float_validator)
        p_inputs.addWidget(self.payload_com_x_entry)

        p_inputs.addWidget(QLabel("Y:"))
        self.payload_com_y_entry = QLineEdit("0.0")
        self.payload_com_y_entry.setFixedWidth(56)
        self.payload_com_y_entry.setAlignment(Qt.AlignCenter)
        self.payload_com_y_entry.setValidator(self.float_validator)
        p_inputs.addWidget(self.payload_com_y_entry)

        p_inputs.addWidget(QLabel("Z:"))
        self.payload_com_z_entry = QLineEdit("0.0")
        self.payload_com_z_entry.setFixedWidth(56)
        self.payload_com_z_entry.setAlignment(Qt.AlignCenter)
        self.payload_com_z_entry.setValidator(self.float_validator)
        p_inputs.addWidget(self.payload_com_z_entry)

        p_inputs.addStretch()

        self.btn_apply_payload = QPushButton("Apply Payload")
        self.btn_apply_payload.setStyleSheet("background-color: #16a085; height: 26px; padding: 4px 10px;")
        self.btn_apply_payload.clicked.connect(self.cmd_apply_payload)
        self.btn_reset_payload = QPushButton("Reset (0 kg)")
        self.btn_reset_payload.setStyleSheet("background-color: #7f8c8d; height: 26px; padding: 4px 10px;")
        self.btn_reset_payload.clicked.connect(self.cmd_reset_payload)
        p_inputs.addWidget(self.btn_apply_payload)
        p_inputs.addWidget(self.btn_reset_payload)
        p_outer.addLayout(p_inputs)

        p_status = QHBoxLayout()
        p_status.setSpacing(10)
        self.lbl_payload_active = QLabel("Configured Payload: 0.000 kg | CoM: [0.0, 0.0, 0.0] mm")
        self.lbl_payload_active.setFont(QFont("Consolas", 9))
        p_status.addWidget(self.lbl_payload_active)

        p_outer.addLayout(p_status)

    def setup_sysid_tab(self, parent):
        tab_layout = QVBoxLayout(parent)
        tab_layout.setContentsMargins(10, 10, 10, 10)
        tab_layout.setSpacing(10)

        ctl_group = QGroupBox("Friction Identification: Alignment and Excitation Bounds")
        tab_layout.addWidget(ctl_group)
        ctl_layout = QVBoxLayout(ctl_group)
        ctl_layout.setSpacing(15)

        # Prep pose sub-group
        prep_widget = QWidget()
        prep_layout = QVBoxLayout(prep_widget)
        prep_layout.setContentsMargins(0, 0, 0, 0)
        prep_layout.addWidget(QLabel("Friction Identification Workspace Center (degrees)", font=QFont("Segoe UI", 11, QFont.Bold)))

        prep_inputs_container = QWidget()
        prep_inputs_layout = QHBoxLayout(prep_inputs_container)
        prep_inputs_layout.setContentsMargins(0, 0, 0, 0)
        prep_inputs_layout.setSpacing(15) # Distance between J1 group and J2 group

        self.sysid_prep_entries = []
        default_prep = [0.0, -30.0, 60.0, 180.0, 0.0, 0.0]
        for i in range(6):
            # Create a tight sub-layout for each Joint label + box
            pair_widget = QWidget()
            pair_layout = QHBoxLayout(pair_widget)
            pair_layout.setContentsMargins(0, 0, 0, 0)
            pair_layout.setSpacing(5)
            
            lbl = QLabel(f"J{i+1}:")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            pair_layout.addWidget(lbl)
            
            entry = QLineEdit(str(default_prep[i]))
            entry.setFixedWidth(65)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            pair_layout.addWidget(entry)
            
            prep_inputs_layout.addWidget(pair_widget)
            self.sysid_prep_entries.append(entry)
            
        prep_inputs_layout.addStretch()
        prep_layout.addWidget(prep_inputs_container)
        ctl_layout.addWidget(prep_widget)

        # Excitation limits sub-group
        limits_widget = QWidget()
        limits_layout = QVBoxLayout(limits_widget)
        limits_layout.setContentsMargins(0, 0, 0, 0)
        limits_layout.addWidget(QLabel("Excitation Limits: Fourier + Low Speed (radians from center)", font=QFont("Segoe UI", 11, QFont.Bold)))

        lim_inputs_container = QWidget()
        lim_inputs_layout = QHBoxLayout(lim_inputs_container)
        lim_inputs_layout.setContentsMargins(0, 0, 0, 0)
        lim_inputs_layout.setSpacing(15)

        self.sysid_limit_entries = []
        default_limits = [0.4, 0.6, 0.5, 1.0, 1.0, 3.0]
        for i in range(6):
            pair_widget = QWidget()
            pair_layout = QHBoxLayout(pair_widget)
            pair_layout.setContentsMargins(0, 0, 0, 0)
            pair_layout.setSpacing(5)
            
            lbl = QLabel(f"J{i+1}:")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            pair_layout.addWidget(lbl)
            
            entry = QLineEdit(str(default_limits[i]))
            entry.setFixedWidth(65)
            entry.setAlignment(Qt.AlignCenter)
            entry.setValidator(self.float_validator)
            entry.setStyleSheet("background-color: #fff9db;")
            pair_layout.addWidget(entry)
            
            lim_inputs_layout.addWidget(pair_widget)
            self.sysid_limit_entries.append(entry)
            
        lim_inputs_layout.addStretch() # Core Fix
        limits_layout.addWidget(lim_inputs_container)
        ctl_layout.addWidget(limits_widget)

        # Trigger Actions
        self.btn_sys_prep = QPushButton("Move to Alignment Pose")
        self.btn_sys_prep.setStyleSheet("background-color: #bdc3c7; color: #2c3e50; font-size: 11pt;")
        self.btn_sys_prep.clicked.connect(self.cmd_sysid_prep_align)
        ctl_layout.addWidget(self.btn_sys_prep)

        self.btn_run_sysid = QPushButton("Start Automated Friction Identification")
        self.btn_run_sysid.setStyleSheet("background-color: #e74c3c; height: 40px; font-size: 12pt;")
        self.btn_run_sysid.clicked.connect(self.cmd_run_sysid)
        ctl_layout.addWidget(self.btn_run_sysid)

        # Output terminal representation
        out_group = QGroupBox("Identified Friction Parameters (YAML)")
        tab_layout.addWidget(out_group)
        out_layout = QVBoxLayout(out_group)
        out_layout.setContentsMargins(10, 15, 10, 10)

        from PyQt5.QtWidgets import QTextEdit
        self.txt_output = QTextEdit()
        self.txt_output.setReadOnly(True)
        self.txt_output.setStyleSheet("""
            background-color: #1e272e; 
            color: #2ecc71; 
            font-family: 'Consolas', 'Ubuntu Mono'; 
            font-size: 11pt;
            border: 1px solid #1a252f;
            border-radius: 4px;
            padding: 5px;
        """)
        out_layout.addWidget(self.txt_output)

        self.btn_copy = QPushButton("Copy parameters to clipboard")
        self.btn_copy.setStyleSheet("background-color: #27ae60; height: 35px;")
        self.btn_copy.clicked.connect(self.cmd_copy_yaml)
        out_layout.addWidget(self.btn_copy)

    def sanitize_angle(self, angle_deg, min_limit, max_limit):
        """
        Modulo wrapping and boundary clamping (Congruence Reduction & Clamping).
        """
        # 1. If already within limits, preserve the original value
        if min_limit <= angle_deg <= max_limit:
            return angle_deg
            
        # 2. Try wrapping to the intuitive [-180, 180] degree range
        wrapped_180 = ((angle_deg + 180.0) % 360.0) - 180.0
        if min_limit <= wrapped_180 <= max_limit:
            return wrapped_180
            
        # 3. Try wrapping to the baseline range [min_limit, min_limit + 360]
        wrapped_min = ((angle_deg - min_limit) % 360.0) + min_limit
        if min_limit <= wrapped_min <= max_limit:
            return wrapped_min
            
        # 4. Fallback: Force clamping to physical boundaries
        return max(min_limit, min(max_limit, angle_deg))

    # === Synchronous slider-entry pairing utility ===
    def connect_joint_pair(self, slider, entry, idx):
        """Map QSlider (integer based) and QLineEdit (float validation) bidirectionally."""
        def slider_moved(val):
            entry.blockSignals(True)
            deg_val = val / 100.0
            entry.setText(f"{deg_val:.2f}")
            entry.blockSignals(False)
            self.exact_q_target[idx] = math.radians(deg_val)
            
        def entry_edited():
            try:
                val = float(entry.text())
                lim = self.j_limits[idx]
                
                # Sanitize and intercept manual user input
                val = self.sanitize_angle(val, lim[0], lim[1])
                
                entry.blockSignals(True)
                entry.setText(f"{val:.2f}")
                entry.blockSignals(False)
                
                slider.blockSignals(True)
                slider.setValue(int(val * 100))
                slider.blockSignals(False)
                
                self.exact_q_target[idx] = math.radians(val)
            except ValueError:
                pass

        slider.valueChanged.connect(slider_moved)
        entry.editingFinished.connect(entry_edited)

    def sync_v_slider_to_entry(self, val):
        self.v_entry.blockSignals(True)
        self.v_entry.setText(f"{val / 100.0:.2f}")
        self.v_entry.blockSignals(False)

    def sync_v_entry_to_slider(self):
        try:
            val = float(self.v_entry.text())
            val = max(0.01, min(val, 1.0))
            self.v_entry.blockSignals(True)
            self.v_entry.setText(f"{val:.2f}")
            self.v_entry.blockSignals(False)
            self.v_slider.blockSignals(True)
            self.v_slider.setValue(int(val * 100))
            self.v_slider.blockSignals(False)
        except ValueError:
            pass

    def sync_a_slider_to_entry(self, val):
        self.a_entry.blockSignals(True)
        self.a_entry.setText(f"{val / 100.0:.2f}")
        self.a_entry.blockSignals(False)

    def sync_a_entry_to_slider(self):
        try:
            val = float(self.a_entry.text())
            val = max(0.01, min(val, 1.0))
            self.a_entry.blockSignals(True)
            self.a_entry.setText(f"{val:.2f}")
            self.a_entry.blockSignals(False)
            self.a_slider.blockSignals(True)
            self.a_slider.setValue(int(val * 100))
            self.a_slider.blockSignals(False)
        except ValueError:
            pass

    # === Banner color transitions ===
    def set_banner_color(self, hex_color):
        self.state_banner.setStyleSheet(f"""
            background-color: {hex_color};
            color: #ffffff;
            border-top-left-radius: 6px;
            border-top-right-radius: 6px;
        """)

    def on_command_error(self, message):
        # Reuse one non-modal notice. exec_() would keep the emitting call's
        # state lock held during a nested Qt event loop, blocking ROS heartbeats
        # while the GUI watchdog continues to run and falsely requests ESTOP.
        summary, _, detail = message.partition("\n")
        if self._command_notice is None:
            self._command_notice = QMessageBox(
                QMessageBox.Warning, "Supervisor / Action Notice", "", QMessageBox.Ok, self)
            self._command_notice.setTextFormat(Qt.PlainText)
            self._command_notice.setWindowModality(Qt.NonModal)
        self._command_notice.setText(summary)
        self._command_notice.setDetailedText(detail)
        self._command_notice.show()

    def on_estop_fault(self, message):
        # Use a dedicated non-modal dialog so the Qt event loop and ROS
        # watchdog continue running while clearly displaying the full cause.
        summary, _, detail = message.partition("\n")
        if self._estop_notice is None:
            self._estop_notice = QMessageBox(
                QMessageBox.Critical, "Emergency Stop (E-STOP) Triggered", "", QMessageBox.Ok, self)
            self._estop_notice.setTextFormat(Qt.PlainText)
            self._estop_notice.setWindowModality(Qt.NonModal)
        self._estop_notice.setText(summary)
        self._estop_notice.setInformativeText(detail)
        self._estop_notice.show()
        self._estop_notice.raise_()
        self._estop_notice.activateWindow()

    def on_recovery_completed(self):
        # Reproduce the original recovery UI synchronization, but only once
        # the Supervisor has observed CTC MODE_NORMAL.
        self.sync_joints()
        self.sync_pose()

    # === Telemetry Sync and UI refresh slots ===
    def on_telemetry_updated(self, pkt):
        """Processes high-frequency ROS 2 telemetry packet in Qt GUI thread."""
        q_deg = [math.degrees(v) for v in pkt['q']]
        for i in range(6):
            self.lbl_q_curr[i].setText(f" J{i+1}: {q_deg[i]:7.2f} °")

        p = pkt['pose']
        self.lbl_p_curr[0].setText(f" X (mm): {p[0]*1000:7.2f}")
        self.lbl_p_curr[1].setText(f" Y (mm): {p[1]*1000:7.2f}")
        self.lbl_p_curr[2].setText(f" Z (mm): {p[2]*1000:7.2f}")
        self.lbl_p_curr[3].setText(f" Roll (°): {math.degrees(p[3]):7.2f}")
        self.lbl_p_curr[4].setText(f" Pitch(°): {math.degrees(p[4]):7.2f}")
        self.lbl_p_curr[5].setText(f" Yaw  (°): {math.degrees(p[5]):7.2f}")

    def on_motion_feedback(self, state_code, pct, exec_time):
        fb_names = {
            IndustrialMotion.Feedback.STATE_PLANNING: "PLANNING",
            IndustrialMotion.Feedback.STATE_WAITING_CTC: "WAITING CTC",
            IndustrialMotion.Feedback.STATE_EXECUTING: "EXECUTING",
            IndustrialMotion.Feedback.STATE_CANCELING: "CANCELING",
        }
        label = fb_names.get(state_code, "MOTION")
        self.bar_motion_progress.setValue(int(round(max(0.0, min(100.0, pct)))))
        if exec_time >= 0.0:
            self.lbl_motion_feedback.setText(f"Motion: DONE ({exec_time:.2f} s)")
        else:
            self.lbl_motion_feedback.setText(f"Motion: {label} ({pct:.0f}%)")

    def cmd_apply_payload(self):
        try:
            mass = float(self.payload_mass_entry.text())
            cx_mm = float(self.payload_com_x_entry.text())
            cy_mm = float(self.payload_com_y_entry.text())
            cz_mm = float(self.payload_com_z_entry.text())
        except ValueError as exc:
            QMessageBox.critical(self, "Payload Input Error", f"Invalid numeric value: {exc}")
            return
        if not (0.0 <= mass <= MAX_PAYLOAD_MASS_KG):
            QMessageBox.warning(
                self, "Payload Out of Range", f"Additional payload must lie within [0.0, {MAX_PAYLOAD_MASS_KG}] kg (simulation limit)."
            )
            return
        com_m = [cx_mm / 1000.0, cy_mm / 1000.0, cz_mm / 1000.0]
        radius = math.hypot(*com_m)
        if not math.isfinite(radius) or radius > MAX_PAYLOAD_COM_RADIUS_M:
            QMessageBox.warning(
                self,
                "Payload CoM Out of Range",
                f"CoM must be finite and within {MAX_PAYLOAD_COM_RADIUS_M * 1000.0:.1f} mm of the TCP.",
            )
            return
        self.worker.set_payload(mass, com_m)

    def cmd_reset_payload(self):
        self.payload_mass_entry.setText("0.000")
        self.payload_com_x_entry.setText("0.0")
        self.payload_com_y_entry.setText("0.0")
        self.payload_com_z_entry.setText("0.0")
        self.worker.set_payload(0.0, [0.0, 0.0, 0.0])

    def refresh_payload_status(self):
        import time
        now = time.monotonic()
        with self.worker.state_lock:
            snap = self.worker.supervisor_state
            state = self.worker.state
            preparation = self.worker.sysid_preparation_status
            fresh = bool(
                snap is not None
                and snap.controller_fresh
                and snap.joint_states_fresh
                and (now - self.worker.last_supervisor_rx_time < 0.8)
            )
        if not fresh or snap is None:
            self.lbl_payload_active.setText("Configured Payload: feedback unavailable")
            self.btn_run_sysid.setEnabled(False)
            return

        mass = float(snap.payload_mass)
        com = list(snap.payload_com_tcp)
        self.lbl_payload_active.setText(
            f"Configured Payload: {mass:.3f} kg | CoM: [{com[0]*1000.0:+.1f}, {com[1]*1000.0:+.1f}, {com[2]*1000.0:+.1f}] mm"
        )
        has_payload = mass > 1e-6
        can_sysid = (state == Lite6StateMachine.IDLE) and (not preparation) and (not has_payload)
        self.btn_run_sysid.setEnabled(can_sysid)
        if has_payload and not preparation:
            self.btn_run_sysid.setText("Friction ID Disabled (Payload > 0 kg)")
            self.btn_run_sysid.setToolTip(
                "Friction Identification is disabled when payload is non-zero. Reset payload to 0 kg first."
            )
        else:
            self.btn_run_sysid.setText(
                preparation if preparation else "Start Automated Friction Identification"
            )
            self.btn_run_sysid.setToolTip("")

    def on_state_updated(self, state, error_msg):
        """Processes state transition events safely."""
        with self.worker.state_lock:
            preparation = self.worker.sysid_preparation_status
            snap = self.worker.supervisor_state
            has_payload = bool(snap is not None and snap.payload_mass > 1e-6)
        self.btn_run_sysid.setEnabled(
            state == Lite6StateMachine.IDLE and not preparation and not has_payload
        )
        if has_payload and not preparation:
            self.btn_run_sysid.setText("Friction ID Disabled (Payload > 0 kg)")
        else:
            self.btn_run_sysid.setText(
                preparation if preparation else "Start Automated Friction Identification")
        # Keep long per-branch diagnostics readable without expanding the banner.
        self.state_banner.setToolTip(error_msg)
        error_summary = error_msg.splitlines()[0] if error_msg else ""
        if len(error_summary) > 140:
            error_summary = error_summary[:137] + "..."
        if state == Lite6StateMachine.IDLE:
            if preparation:
                self.set_banner_color("#8e44ad")
                self.state_banner.setText(f"FRICTION ID PREPARATION: {preparation}")
            else:
                self.set_banner_color("#27ae60") # Emerald
                self.state_banner.setText("SYSTEM READY")
        elif state == Lite6StateMachine.ERROR:
            self.set_banner_color("#c0392b") # Alizarin red
            self.state_banner.setText(f"SYSTEM FAULT: {error_summary}")
        elif state == Lite6StateMachine.SOFT_ERROR:
            self.set_banner_color("#d35400") # Pumpkin Orange
            self.state_banner.setText(f"WARNING: {error_summary}")
        elif state.startswith("FRICTION ID"):
            self.set_banner_color("#8e44ad") # Amethyst purple
            self.state_banner.setText(state)
        else:
            self.set_banner_color("#2980b9") # Peter river blue
            self.state_banner.setText(state)

    def on_sysid_yaml_updated(self, yaml_string):
        self.txt_output.clear()
        self.txt_output.insertPlainText(yaml_string)

    def on_sysid_preparation_updated(self):
        # Read current values so a queued progress notification cannot overwrite
        # a newer fault/mode or restore an obsolete preparation message.
        if self._supervisor_offline_displayed:
            return
        with self.worker.state_lock:
            state, detail = self.worker.state, self.worker.error_msg
        self.on_state_updated(state, detail)

    def check_connection_watchdog(self):
        """Monitor supervisor feedback and send direct CTC E-STOP if it disappears."""
        offline = self.worker.check_watchdog()
        if offline:
            if not self._supervisor_offline_displayed:
                self._supervisor_offline_displayed = True
                self.set_banner_color("#c0392b")
                self.state_banner.setText("SUPERVISOR OFFLINE - E-STOP FALLBACK")
        elif self._supervisor_offline_displayed:
            self._supervisor_offline_displayed = False
            with self.worker.state_lock:
                state = self.worker.state
                detail = self.worker.error_msg
            self.on_state_updated(state, detail)

    # === Synchronizers ===
    def sync_joints(self):
        with self.worker.state_lock:
            q_copied = list(self.worker.q_curr)
            
        for i in range(6):
            lim = self.j_limits[i]
            raw_deg = math.degrees(q_copied[i])
            
            # Execute congruence reduction and filtering
            deg_val = self.sanitize_angle(raw_deg, lim[0], lim[1])
            
            # Update target registry memory (in radians)
            self.exact_q_target[i] = math.radians(deg_val)
            
            # Update HMI display fields
            self.j_entries[i].blockSignals(True)
            self.j_entries[i].setText(f"{deg_val:.2f}")
            self.j_entries[i].blockSignals(False)
            
            self.j_sliders[i].blockSignals(True)
            self.j_sliders[i].setValue(int(deg_val * 100))
            self.j_sliders[i].blockSignals(False)

    def _sync_coordinates_to_entries(self, exact_storage, entries):
        with self.worker.state_lock:
            p = list(self.worker.pose_curr)
            
        for i in range(6): exact_storage[i] = p[i]
        entries[0].setText(f"{p[0]*1000:.2f}")
        entries[1].setText(f"{p[1]*1000:.2f}")
        entries[2].setText(f"{p[2]*1000:.2f}")
        entries[3].setText(f"{math.degrees(p[3]):.2f}")
        entries[4].setText(f"{math.degrees(p[4]):.2f}")
        entries[5].setText(f"{math.degrees(p[5]):.2f}")

    def sync_pose(self):
        self._sync_coordinates_to_entries(self.exact_p_target, self.p_entries)

    def sync_aux(self):
        self._sync_coordinates_to_entries(self.exact_aux_target, self.aux_p_entries)

    # === Command Dispatchers ===
    def cmd_move_j(self):
        try:
            target_rad = [0.0] * 6
            for i in range(6):
                ui_val = float(self.j_entries[i].text())
                lim = self.j_limits[i]
                
                # Defensive clamping check
                clamped_val = self.sanitize_angle(ui_val, lim[0], lim[1])
                if abs(ui_val - clamped_val) > 1e-4:
                    self.worker.node.get_logger().warn(
                        f"J{i+1} target {ui_val}° violated limits. Sanitized to {clamped_val}°"
                    )
                    ui_val = clamped_val
                    self.j_entries[i].setText(f"{clamped_val:.2f}")
                    self.j_sliders[i].setValue(int(clamped_val * 100))
                
                ui_str_of_exact = float(f"{math.degrees(self.exact_q_target[i]):.2f}")
                if abs(ui_val - ui_str_of_exact) < 1e-6:
                    target_rad[i] = self.exact_q_target[i]
                else:
                    target_rad[i] = math.radians(ui_val)
                    self.exact_q_target[i] = target_rad[i]
            
            self.worker.execute_joint_target(target_rad, self.v_var_val(), self.a_var_val())
        except Exception as e:
            QMessageBox.critical(self, "Command Fault", f"Input format parse exception: {e}")

    def _parse_cartesian_entries(self, entries, exact_storage):
        target = [0.0] * 6
        
        for i in range(3):
            ui_val = float(entries[i].text())
            ui_str_of_exact = float(f"{exact_storage[i] * 1000.0:.2f}")
            
            if abs(ui_val - ui_str_of_exact) < 1e-6:
                target[i] = exact_storage[i]
            else:
                target[i] = ui_val / 1000.0
                exact_storage[i] = target[i]
                
        for i in range(3, 6):
            ui_val = float(entries[i].text())
            ui_str_of_exact = float(f"{math.degrees(exact_storage[i]):.2f}")
            
            if abs(ui_val - ui_str_of_exact) < 1e-6:
                target[i] = exact_storage[i]
            else:
                # Wrap angle to [-180, 180] degrees before converting to radians
                wrapped_val = ((ui_val + 180.0) % 360.0) - 180.0
                target[i] = math.radians(wrapped_val)
                exact_storage[i] = target[i]

                entries[i].blockSignals(True)
                entries[i].setText(f"{wrapped_val:.2f}")
                entries[i].blockSignals(False)
                
        return target

    def cmd_move_p(self):
        try:
            target = self._parse_cartesian_entries(self.p_entries, self.exact_p_target)
            self.worker.execute_pose_target(target, self.v_var_val(), self.a_var_val(), IndustrialMotion.Goal.MOVEP)
        except Exception as e:
            QMessageBox.critical(self, "Command Fault", f"Invalid input parsed: {e}")

    def cmd_move_l(self):
        try:
            target = self._parse_cartesian_entries(self.p_entries, self.exact_p_target)
            self.worker.execute_pose_target(target, self.v_var_val(), self.a_var_val(), IndustrialMotion.Goal.MOVEL)
        except Exception as e:
            QMessageBox.critical(self, "Command Fault", f"Invalid input parsed: {e}")

    def cmd_move_c(self):
        try:
            target = self._parse_cartesian_entries(self.p_entries, self.exact_p_target)
            aux_target = self._parse_cartesian_entries(self.aux_p_entries, self.exact_aux_target)
            self.worker.execute_circular_target(target, aux_target, self.v_var_val(), self.a_var_val())
        except Exception as e:
            QMessageBox.critical(self, "Command Fault", f"Circular geometry definition incorrect: {e}")

    def cmd_home(self):
        home_deg = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
        for i in range(6):
            self.j_entries[i].setText(f"{home_deg[i]:.2f}")
            self.j_sliders[i].setValue(int(home_deg[i] * 100))
        self.cmd_move_j()

    # === Automated Friction Identification Sequencer Calls ===
    def _parse_sysid_inputs(self):
        prep_pose = []
        for i, entry in enumerate(self.sysid_prep_entries):
            val = float(entry.text())
            lim = self.j_limits[i]
            sanitized = self.sanitize_angle(val, lim[0], lim[1])
            if abs(val - sanitized) > 1e-4:
                entry.setText(f"{sanitized:.2f}")
                val = sanitized
            prep_pose.append(math.radians(val))
            
        max_limits = [float(entry.text()) for entry in self.sysid_limit_entries]
        return prep_pose, max_limits

    def cmd_sysid_prep_align(self):
        with self.worker.state_lock:
            state = self.worker.state
        if state != Lite6StateMachine.IDLE:
            QMessageBox.warning(self, "Command Blocked", "System must be IDLE.")
            return
        try:
            prep_pose, _ = self._parse_sysid_inputs()
            self.worker.execute_joint_target(prep_pose, 1.00, 1.00)
        except Exception as e:
            QMessageBox.critical(self, "Input parsing fault", f"Error mapping configuration: {e}")

    def cmd_run_sysid(self):
        with self.worker.state_lock:
            state = self.worker.state
            q_curr_copy = list(self.worker.q_curr)
            
        if state != Lite6StateMachine.IDLE:
            QMessageBox.warning(self, "Command Blocked", "Robot system must be IDLE.")
            return
        
        try:
            prep_pose, max_limits = self._parse_sysid_inputs()
        except Exception as e:
            QMessageBox.critical(self, "Input Parse Error", f"Failed to parse inputs: {e}")
            return

        # Check alignment offset to ensure workspace center synchronization
        current_offset = np.abs(np.array(q_curr_copy) - np.array(prep_pose)).max()
        if current_offset > 0.02:
            res = QMessageBox.question(
                self, "Friction Identification Alignment",
                "Robot is not aligned with workspace center.\nPlan and align now?",
                QMessageBox.Yes | QMessageBox.No
            )
            if res == QMessageBox.Yes:
                self.worker.start_sysid_sequence(prep_pose, max_limits)
            return

        self.worker.start_sysid_sequence(prep_pose, max_limits)

    def cmd_copy_yaml(self):
        raw_text = self.txt_output.toPlainText().strip()
        if not raw_text:
            QMessageBox.warning(self, "Clipboard Warning", "Terminal box empty.")
            return
        clipboard = QApplication.clipboard()
        clipboard.setText(raw_text)
        QMessageBox.information(self, "Success", "Identified parameters copied to clipboard.")

    def gui_reset_fault(self):
        # Request a supervised event, never force IDLE or NORMAL locally.
        with self.worker.state_lock:
            state = self.worker.state
        if state == Lite6StateMachine.SOFT_ERROR:
            self.worker.acknowledge_soft_fault()
        elif state == Lite6StateMachine.ERROR:
            self.worker.begin_recovery()

    # === Helper scaling utilities ===
    def v_var_val(self):
        return float(self.v_entry.text())

    def a_var_val(self):
        return float(self.a_entry.text())

    def on_closing(self):
        self.close()


def main(args=None):
    os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "1.0"
    os.environ["QT_SCALE_FACTOR"] = "1.0"    # This factor changes the total size of the GUI window. The fonts inside it will be changed automatically.
    rclpy.init(args=args)

    # Enable High-DPI Scaling for 2K/4K screens
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    try:
        QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    except AttributeError:
        pass # Ignored if using an older version of PyQt5

    # 2. Instantiate and launch the main Qt Application loop
    app = QApplication(sys.argv)
    
    # Establish dynamic color palette settings matching modern OS environments
    app.setStyle('Fusion')
    
    # Connect every GUI receiver before the executor can emit its first state.
    worker = ROS2Worker()
    gui = PyQtHMI(worker)
    gui.show()
    worker.start()

    # Allow Ctrl+C / launch SIGINT to exit the Qt event loop gracefully.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sig_timer = QTimer()
    sig_timer.start(200)
    sig_timer.timeout.connect(lambda: None)

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        sig_timer.stop()
        # Graceful shutdown of the ROS executor thread before destroying node.
        worker.stop()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
