"""Entry point for Lite6 Studio HMI Qt5 application."""

import signal
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
import rclpy

from lite6_studio_hmi.studio_ros_client import StudioRosWorker
from lite6_studio_hmi.studio_window import StudioMainWindow


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setApplicationName("Lite6 Industrial Studio")

    worker = StudioRosWorker()
    window = StudioMainWindow(worker)
    worker.start()
    window.show()

    # Allow Ctrl+C from terminal launch to exit Qt event loop gracefully
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sig_timer = QTimer()
    sig_timer.start(200)
    sig_timer.timeout.connect(lambda: None)

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        sig_timer.stop()
        worker.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
