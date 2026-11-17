"""Independent compliance and encoder teaching panel."""
import json
import math
import time
from pathlib import Path
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel,
                             QComboBox, QDoubleSpinBox, QCheckBox, QPushButton,
                             QGroupBox, QFileDialog, QMessageBox)
from lite6_interfaces.msg import ComplianceSettings, SupervisorState as S
from lite6_interfaces.srv import SupervisorCommand as C


class ComplianceTab(QWidget):
    _command_done_signal = pyqtSignal()

    def __init__(self, worker, parent=None):
        super().__init__(parent)
        self.worker = worker
        self.pending = False
        self._command_done_signal.connect(self._on_command_done)
        layout = QVBoxLayout(self)
        info = QLabel("Encoder-based compliance + passive friction feedforward + sensorless GMO load observer at link_eef. "
                      "Entering a mode captures the current equilibrium.\n"
                      "In the viewer: select a link, then Ctrl + right drag for force or Ctrl + left drag for torque.")
        info.setWordWrap(True)
        layout.addWidget(info)
        self.mode = QComboBox()
        self.mode.addItems(["Joint-space impedance", "Cartesian-space impedance", "Drag teaching"])
        layout.addWidget(self.mode)
        self.fields = {}
        self.groups = []
        definitions = [
            ("Joint gains", [("joint_stiffness", "Stiffness (Nm/rad)", 0, 100, 20),
                             ("joint_damping", "Damping (Nm s/rad)", .1, 20, 3)]),
            ("Cartesian gains (world-aligned)", [
                ("translation_stiffness", "Translation stiffness (N/m)", 0, 1000, 200),
                ("translation_damping", "Translation damping (N s/m)", 1, 100, 25),
                ("rotation_stiffness", "Rotation stiffness (Nm/rad)", 0, 50, 8),
                ("rotation_damping", "Rotation damping (Nm s/rad)", .1, 20, 1.5)]),
            ("Teaching sensitivity", [("sensitivity", "1 = firm, 5 = light", 1, 5, 3)])]
        for title, fields in definitions:
            group = QGroupBox(title)
            form = QFormLayout(group)
            for name, label, low, high, value in fields:
                field = QDoubleSpinBox()
                field.setRange(low, high)
                field.setDecimals(2)
                field.setValue(value)
                field.setSingleStep(.1 if high <= 20 else 1)
                form.addRow(label, field)
                self.fields[name] = field
            layout.addWidget(group)
            self.groups.append(group)
        self.recapture = QCheckBox("Capture current equilibrium when applying in the same mode")
        layout.addWidget(self.recapture)
        actions = QHBoxLayout()
        self.apply = QPushButton("Enter / Apply settings")
        self.apply.clicked.connect(self.send)
        self.exit = QPushButton("Stop and return to NORMAL")
        self.exit.clicked.connect(self.leave)
        actions.addWidget(self.apply)
        actions.addWidget(self.exit)
        layout.addLayout(actions)
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        observer = QGroupBox("Sensorless External Load Observer (GMO)")
        obs_layout = QVBoxLayout(observer)
        obs_head = QHBoxLayout()
        self.ext_wrench_label = QLabel("TCP Force:  0.0 N  [+0.0, +0.0, +0.0]  |  Moment: [+0.00, +0.00, +0.00] Nm")
        self.ext_wrench_label.setFont(QFont("Consolas", 9))
        self.ext_wrench_label.setWordWrap(True)
        self.contact_badge = QLabel("NO CONTACT")
        self.contact_badge.setStyleSheet(
            "background-color: #E2E8F0; color: #334155; font-weight: bold; "
            "border-radius: 4px; padding: 3px 8px;"
        )
        obs_head.addWidget(self.ext_wrench_label, 1)
        obs_head.addWidget(self.contact_badge)
        obs_layout.addLayout(obs_head)
        self.ext_tau_label = QLabel("Ext Joint Tau (Nm): [ +0.00,  +0.00,  +0.00,  +0.00,  +0.00,  +0.00]")
        self.ext_tau_label.setFont(QFont("Consolas", 9))
        self.ext_tau_label.setWordWrap(True)
        obs_layout.addWidget(self.ext_tau_label)
        layout.addWidget(observer)
        teaching = QGroupBox("Teaching capture (encoder positions only)")
        capture = QVBoxLayout(teaching)
        row = QHBoxLayout()
        self.point = QPushButton("Capture point")
        self.point.clicked.connect(self.capture_point)
        self.record = QPushButton("Start recording")
        self.record.clicked.connect(self.toggle_recording)
        self.clear = QPushButton("Clear captured data")
        self.clear.clicked.connect(self.clear_data)
        self.export = QPushButton("Export JSON")
        self.export.clicked.connect(self.export_data)
        for button in (self.point, self.record, self.clear, self.export):
            row.addWidget(button)
        capture.addLayout(row)
        self.record_status = QLabel()
        capture.addWidget(self.record_status)
        note = QLabel("Release the arm before switching modes. Higher sensitivity reduces damping; levels 1-5 fade out Coulomb assistance near rest.\n"
                      "Static friction remains; zero-stiffness teaching does not actively hold position. Use Stop to hold.\n"
                      "Captured paths require smoothing, retiming and validation before any future playback.")
        note.setWordWrap(True)
        capture.addWidget(note)
        layout.addWidget(teaching)
        layout.addStretch()
        self.mode.currentIndexChanged.connect(self.select_mode)
        self.select_mode()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.refresh()

    def select_mode(self, *_):
        for i, group in enumerate(self.groups):
            group.setVisible(i == self.mode.currentIndex())
        self.recapture.setVisible(self.mode.currentIndex() != 2)

    def send(self):
        settings = ComplianceSettings()
        for name, field in self.fields.items():
            setattr(settings, name, field.value())
        settings.recapture = self.recapture.isChecked()
        self.request([C.Request.CMD_JOINT_IMPEDANCE, C.Request.CMD_CARTESIAN_IMPEDANCE,
                      C.Request.CMD_TEACH][self.mode.currentIndex()], settings)

    def leave(self):
        self.request(C.Request.CMD_EXIT_COMPLIANCE)

    def _on_command_done(self):
        self.pending = False
        self.refresh()

    def request(self, event, settings=None):
        self.pending = True
        def done(_):
            # ROS callback runs on the executor thread; marshal completion to Qt thread.
            self._command_done_signal.emit()
        if not self.worker._send_command(event, compliance=settings, done=done):
            self.pending = False
        self.refresh()

    def refresh(self):
        with self.worker.state_lock:
            state = self.worker.supervisor_state
            fresh = bool(state and state.controller_fresh and state.joint_states_fresh and
                         time.monotonic() - self.worker.last_supervisor_rx_time < .8 and
                         time.monotonic() - self.worker.last_state_rx_time < .25)
            active = bool(state and state.mode in (S.MODE_JOINT_IMPEDANCE, S.MODE_CARTESIAN_IMPEDANCE, S.MODE_TEACH))
            ready = fresh and not state.estop_latched and not self.pending
            self.apply.setEnabled(bool(ready and (active or state.motion_allowed)))
            self.exit.setEnabled(bool(ready and active))
            teach = fresh and state.mode == S.MODE_TEACH
            recording = self.worker.teaching
            if not teach:
                recording.stop()
            self.point.setEnabled(bool(teach))
            self.record.setEnabled(bool(teach))
            self.record.setText("Stop recording" if recording.active else "Start recording")
            self.export.setEnabled(bool(recording.points or recording.samples) and not recording.active)
            self.clear.setEnabled(not recording.active and bool(recording.points or recording.samples))
            self.record_status.setText(f"Points: {len(recording.points)} | Samples: {len(recording.samples)} | {recording.note}")
            labels = {S.MODE_JOINT_IMPEDANCE: "Joint impedance", S.MODE_CARTESIAN_IMPEDANCE: "Cartesian impedance",
                      S.MODE_TEACH: "Drag teaching", S.MODE_COMPLIANCE_SWITCHING: "Switching / stopping",
                      S.MODE_NORMAL: "NORMAL", S.MODE_ESTOP: "E-STOP"}
            detail_summary = state.detail.splitlines()[0] if (fresh and state.detail) else ""
            self.status.setToolTip(state.detail if fresh else "")
            self.status.setText((f"Observed mode: {labels.get(state.mode, str(state.mode))} | {detail_summary}"
                                 if fresh else "Waiting for fresh Supervisor / encoder feedback"))
            if fresh:
                w = state.external_cartesian_wrench
                tau = state.external_joint_torque
                f_norm = math.sqrt(w[0] ** 2 + w[1] ** 2 + w[2] ** 2)
                self.ext_wrench_label.setText(
                    f"TCP Force: {f_norm:5.1f} N [{w[0]:+5.1f}, {w[1]:+5.1f}, {w[2]:+5.1f}]  |  "
                    f"Moment: [{w[3]:+5.2f}, {w[4]:+5.2f}, {w[5]:+5.2f}] Nm"
                )
                tau_str = ", ".join(f"{v:+6.2f}" for v in tau)
                self.ext_tau_label.setText(f"Ext Joint Tau (Nm): [{tau_str}]")
                if state.contact_detected:
                    self.contact_badge.setText("CONTACT")
                    self.contact_badge.setStyleSheet(
                        "background-color: #FEF3C7; color: #B45309; font-weight: bold; "
                        "border-radius: 4px; padding: 3px 8px;"
                    )
                else:
                    self.contact_badge.setText("NO CONTACT")
                    self.contact_badge.setStyleSheet(
                        "background-color: #E2E8F0; color: #334155; font-weight: bold; "
                        "border-radius: 4px; padding: 3px 8px;"
                    )
            else:
                self.ext_wrench_label.setText("TCP Force:  -- N  |  Moment: -- Nm")
                self.ext_tau_label.setText("Ext Joint Tau (Nm): [--]")
                self.contact_badge.setText("OFFLINE")
                self.contact_badge.setStyleSheet(
                    "background-color: #E2E8F0; color: #64748B; font-weight: bold; "
                    "border-radius: 4px; padding: 3px 8px;"
                )

    def capture_point(self):
        with self.worker.state_lock:
            if not self.worker.supervisor_state or self.worker.supervisor_state.mode != S.MODE_TEACH:
                return
            self.worker.teaching.points.append({"position_rad": list(self.worker.q_curr)})
        self.refresh()

    def toggle_recording(self):
        with self.worker.state_lock:
            recorder = self.worker.teaching
            if recorder.active:
                recorder.stop()
            else:
                try:
                    recorder.start()
                except ValueError as exc:
                    self.worker._report_error(str(exc))
        self.refresh()

    def clear_data(self):
        if QMessageBox.question(self, "Clear teaching data", "Discard all captured points and samples?") != QMessageBox.Yes:
            return
        with self.worker.state_lock:
            self.worker.teaching.samples.clear()
            self.worker.teaching.points.clear()
            self.worker.teaching.note = "Data cleared"
        self.refresh()

    def export_data(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export teaching data", "teaching.json", "JSON (*.json)")
        if not path:
            return
        with self.worker.state_lock:
            document = self.worker.teaching.document()
        try:
            Path(path).write_text(json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        except (OSError, ValueError) as exc:
            self.worker._report_error(f"Teaching export failed: {exc}")
