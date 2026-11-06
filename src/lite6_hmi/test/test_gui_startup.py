"""Exercise Qt startup offscreen without creating ROS nodes or issuing motion."""
import os
import threading

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QApplication
from lite6_hmi import gui_main
from lite6_hmi.supervisor_worker import Lite6StateMachine


class WorkerSignals(QObject):
    telemetry_signal = pyqtSignal(dict)
    state_signal = pyqtSignal(str, str)
    sysid_yaml_signal = pyqtSignal(str)
    command_error_signal = pyqtSignal(str)
    recovery_complete_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.state_lock = threading.RLock()
        self.state = Lite6StateMachine.IDLE
        self.error_msg = ''

    def check_watchdog(self):
        return False


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


def test_ready_received_before_gui_is_rendered(app):
    worker = WorkerSignals()
    worker.state_signal.emit(worker.state, '')  # No receiver yet.
    gui = gui_main.PyQtHMI(worker)
    try:
        assert gui.state_banner.text() == 'SYSTEM READY'
        worker.state_signal.emit(Lite6StateMachine.PLANNING, '')
        assert gui.state_banner.text() == Lite6StateMachine.PLANNING
    finally:
        gui.watchdog_timer.stop()
        gui.close()


def test_main_connects_state_receiver_before_starting_worker(app, monkeypatch):
    started = []

    class Worker(WorkerSignals):
        def start(self):
            assert QApplication.instance() is not None
            assert self.receivers(self.state_signal) > 0
            started.append(True)
            self.state_signal.emit(self.state, '')

        def stop(self):
            pass

    monkeypatch.setattr(gui_main, 'ROS2Worker', Worker)
    monkeypatch.setattr(gui_main, 'QApplication', lambda args: app)
    # The production startup configures these static Qt attributes first.
    gui_main.QApplication.setAttribute = lambda *args: None
    gui_main.QApplication.setHighDpiScaleFactorRoundingPolicy = lambda *args: None
    monkeypatch.setattr(app, 'exec_', lambda: 0)
    monkeypatch.setattr(gui_main.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(gui_main.rclpy, 'shutdown', lambda: None)
    with pytest.raises(SystemExit) as exc:
        gui_main.main([])
    assert exc.value.code == 0
    assert started == [True]
