"""Entry point for Lite6 Studio HMI Qt5 application."""

import signal
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtWidgets import QApplication
import rclpy

from lite6_studio_hmi.studio_ros_client import StudioRosWorker
from lite6_studio_hmi.studio_window import StudioMainWindow


def _apply_light_palette(app):
    """Apply uniform Fusion Light palette so desktop GTK themes never override contrast."""
    app.setStyle("Fusion")
    app.setFont(QFont("Noto Sans", 10))

    pal = QPalette()
    pal.setColor(QPalette.Window, QColor("#F3F5F8"))
    pal.setColor(QPalette.WindowText, QColor("#0F172A"))
    pal.setColor(QPalette.Base, QColor("#FFFFFF"))
    pal.setColor(QPalette.AlternateBase, QColor("#F8FAFC"))
    pal.setColor(QPalette.ToolTipBase, QColor("#FFFFFF"))
    pal.setColor(QPalette.ToolTipText, QColor("#0F172A"))
    pal.setColor(QPalette.Text, QColor("#0F172A"))
    pal.setColor(QPalette.Button, QColor("#FFFFFF"))
    pal.setColor(QPalette.ButtonText, QColor("#1E293B"))
    pal.setColor(QPalette.BrightText, QColor("#DC2626"))
    pal.setColor(QPalette.Highlight, QColor("#DBEAFE"))
    pal.setColor(QPalette.HighlightedText, QColor("#1E3A8A"))
    pal.setColor(QPalette.Disabled, QPalette.Text, QColor("#94A3B8"))
    pal.setColor(QPalette.Disabled, QPalette.ButtonText, QColor("#94A3B8"))
    app.setPalette(pal)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    app.setApplicationName("Lite6 Industrial Studio")
    _apply_light_palette(app)

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
