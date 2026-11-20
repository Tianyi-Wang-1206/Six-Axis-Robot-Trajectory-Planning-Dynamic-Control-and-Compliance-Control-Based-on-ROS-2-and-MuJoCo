"""Independent compliance and encoder teaching panel."""
import json
import time
from pathlib import Path
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QGridLayout, QLabel,
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
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(6)
        info = QLabel("Encoder-based impedance and gravity-compensated teaching at link_eef. "
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
            ("Cartesian gains (world-aligned, equal damping ratio)", [
                ("translation_stiffness", "Translation stiffness (N/m)", 0, 1000, 200),
                ("translation_damping", "Translation damping ratio (ζ_t)", .1, 2.0, .8),
                ("rotation_stiffness", "Rotation stiffness (Nm/rad)", 0, 50, 8),
                ("rotation_damping", "Rotation damping ratio (ζ_r)", .1, 2.0, .8),
                ("nullspace_stiffness", "Null-space stiffness (Nm/rad)", 0, 50, 0.0),
                ("nullspace_damping", "Null-space damping (Nm s/rad)", 0, 20, 0.5)]),
            ("Teaching sensitivity (diag(M)-normalized)", [("sensitivity", "1 = firm, 5 = light", 1, 5, 3)])]
        for title, fields in definitions:
            group = QGroupBox(title)
            if len(fields) >= 6:
                grid = QGridLayout(group)
                grid.setHorizontalSpacing(10)
                grid.setVerticalSpacing(6)
                for idx, (name, label, low, high, value) in enumerate(fields):
                    field = QDoubleSpinBox()
                    field.setRange(low, high)
                    field.setDecimals(2)
                    field.setValue(value)
                    field.setSingleStep(.1 if high <= 20 else 1)
                    field.setMinimumHeight(26)
                    row_i = idx // 2
                    col_i = (idx % 2) * 2
                    grid.addWidget(QLabel(label), row_i, col_i)
                    grid.addWidget(field, row_i, col_i + 1)
                    self.fields[name] = field
            else:
                form = QFormLayout(group)
                for name, label, low, high, value in fields:
                    field = QDoubleSpinBox()
                    field.setRange(low, high)
                    field.setDecimals(2)
                    field.setValue(value)
                    field.setSingleStep(.1 if high <= 20 else 1)
                    field.setMinimumHeight(26)
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
        teaching = QGroupBox("Teaching capture (encoder positions only)")
        capture = QVBoxLayout(teaching)
        capture.setContentsMargins(10, 6, 10, 8)
        capture.setSpacing(4)
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
        note = QLabel("Release the arm before switching modes. Higher sensitivity reduces damping.\n"
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
        target_cmd = [C.Request.CMD_JOINT_IMPEDANCE, C.Request.CMD_CARTESIAN_IMPEDANCE,
                      C.Request.CMD_TEACH][self.mode.currentIndex()]
        expected_rev = 0
        with self.worker.state_lock:
            state = self.worker.supervisor_state
            if state is not None:
                expected_rev = int(state.revision)
                same_mode = (
                    (target_cmd == C.Request.CMD_JOINT_IMPEDANCE and state.mode == S.MODE_JOINT_IMPEDANCE) or
                    (target_cmd == C.Request.CMD_CARTESIAN_IMPEDANCE and state.mode == S.MODE_CARTESIAN_IMPEDANCE) or
                    (target_cmd == C.Request.CMD_TEACH and state.mode == S.MODE_TEACH)
                )
                if same_mode:
                    target_cmd = C.Request.CMD_UPDATE_COMPLIANCE
        self.request(target_cmd, settings, expected_rev)

    def leave(self):
        expected_rev = 0
        with self.worker.state_lock:
            if self.worker.supervisor_state is not None:
                expected_rev = int(self.worker.supervisor_state.revision)
        self.request(C.Request.CMD_EXIT_COMPLIANCE, expected_revision=expected_rev)

    def _on_command_done(self):
        self.pending = False
        self.refresh()

    def request(self, event, settings=None, expected_revision=0):
        self.pending = True
        def done(_):
            # ROS callback runs on the executor thread; marshal completion to Qt thread.
            self._command_done_signal.emit()
        if not self.worker._send_command(event, compliance=settings,
                                         expected_revision=expected_revision, done=done):
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
