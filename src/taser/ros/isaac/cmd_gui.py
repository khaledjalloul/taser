#!/usr/bin/env python3
# Requires: PyQt6, rclpy, geometry_msgs, nav_msgs, tf2_ros

import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from PyQt6 import QtCore, QtGui, QtWidgets
from rclpy import node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Int32
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from taser.common.datatypes import Workspace

# ---------------- Helpers ----------------

sys.stdout.reconfigure(line_buffering=True)

LEFT_PANEL_SIZE = 600
CONTROL_WIDTH = 300
HEIGHT = 600

MAP_TOPIC = "/taser/navigation/occupancy_grid"  # hardcoded


def frame_to_robot(x_px: float, y_px: float, fw: int, fh: int, ws: Workspace):
    """Frame (pixels, y-down) -> world (meters, y-up)."""
    xr = (x_px / fw) * (ws.x_max - ws.x_min) + ws.x_min
    yr = ((fh - y_px) / fh) * (ws.y_max - ws.y_min) + ws.y_min
    return xr, yr


def robot_to_frame(xr: float, yr: float, fw: int, fh: int, ws: Workspace):
    """World (meters, y-up) -> frame (pixels, y-down)."""
    x_px = (xr - ws.x_min) / (ws.x_max - ws.x_min) * fw
    y_px = fh - ((yr - ws.y_min) / (ws.y_max - ws.y_min) * fh)
    return x_px, y_px


# ---------------- Canvas ----------------


class CanvasWidget(QtWidgets.QWidget):
    """
    Canvas with three layers:
      - Background pixmap (occupancy grid or placeholder)
      - Robot overlay (circle)
      - Foreground (interactive dot/path)
    Modes:
      - 'click' : single click -> pose
      - 'drag'  : press-move-release -> velocities (unused now)
    """

    def __init__(
        self, width: int, height: int, ws: Workspace | None, mode: str, parent=None
    ):
        super().__init__(parent)
        self.setFixedSize(width, height)

        # Layers
        self._bg_pixmap = QtGui.QPixmap(width, height)
        self._fg_pixmap = QtGui.QPixmap(width, height)
        self._bg_pixmap.fill(QtGui.QColor("white"))
        self._fg_pixmap.fill(QtGui.QColor(0, 0, 0, 0))

        # “Waiting for map…” overlay
        self._waiting_text = None  # str | None

        # Robot overlay
        self._robot_px: QtCore.QPointF | None = None
        self._robot_radius_px: float = 6.0
        self._robot_color = QtGui.QColor("#D81B60")
        self._robot_edge = QtGui.QColor("#880E4F")

        self.setMouseTracking(True)
        self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.CrossCursor))

        self.mode = mode
        self.ws = ws
        self.on_click = None
        self.on_velocity = None  # kept for compatibility, unused now

        # Drag state
        self._pressed = False
        self._last_pos = None
        self._last_time = None

        # Auto-zero velocity when idle (unused, harmless)
        self._zero_timer = QtCore.QTimer(self)
        self._zero_timer.setSingleShot(True)
        self._zero_timer.timeout.connect(self._emit_zero_velocity)
        self._zero_interval_ms = 120

        self.installEventFilter(self)

    # -------- Background --------
    def set_bg_pixmap(self, pixmap: QtGui.QPixmap | None):
        if pixmap is None:
            self._bg_pixmap.fill(QtGui.QColor("white"))
        else:
            self._bg_pixmap = pixmap.scaled(
                self.width(),
                self.height(),
                QtCore.Qt.AspectRatioMode.IgnoreAspectRatio,
                QtCore.Qt.TransformationMode.FastTransformation,
            )
        self.update()

    def set_waiting_text(self, text: str | None):
        self._waiting_text = text
        self.update()

    # -------- Robot overlay --------
    def set_robot_world(self, xr: float, yr: float):
        if self.ws is None:
            return
        x_px, y_px = robot_to_frame(xr, yr, self.width(), self.height(), self.ws)
        self._robot_px = QtCore.QPointF(x_px, y_px)
        self.update()

    # -------- Foreground (interactive) --------
    def clear(self):
        self._fg_pixmap.fill(QtGui.QColor(0, 0, 0, 0))
        self.update()

    def _draw_line(self, p0: QtCore.QPointF, p1: QtCore.QPointF, color="#0099FF"):
        painter = QtGui.QPainter(self._fg_pixmap)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        pen = QtGui.QPen(QtGui.QColor(color))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawLine(p0, p1)
        painter.end()
        self.update()

    def _draw_dot(self, p: QtCore.QPointF, color="#FF9900", radius=4):
        painter = QtGui.QPainter(self._fg_pixmap)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        qcolor = QtGui.QColor(color)
        painter.setBrush(qcolor)
        painter.setPen(QtGui.QPen(qcolor))
        painter.drawEllipse(p, radius, radius)
        painter.end()
        self.update()

    def paintEvent(self, _):
        painter = QtGui.QPainter(self)
        painter.drawPixmap(0, 0, self._bg_pixmap)

        # Robot overlay
        if self._robot_px is not None and self.ws is not None:
            painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
            painter.setBrush(QtGui.QBrush(self._robot_color))
            pen = QtGui.QPen(self._robot_edge)
            pen.setWidth(2)
            painter.setPen(pen)
            painter.drawEllipse(
                self._robot_px, self._robot_radius_px, self._robot_radius_px
            )

        painter.drawPixmap(0, 0, self._fg_pixmap)

        if self._waiting_text:
            painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
            painter.setPen(QtGui.QPen(QtGui.QColor("#666")))
            font = painter.font()
            font.setPointSize(14)
            painter.setFont(font)
            rect = QtCore.QRect(0, 0, self.width(), self.height())
            painter.drawText(
                rect, QtCore.Qt.AlignmentFlag.AlignCenter, self._waiting_text
            )

    # ---- Events ----
    def eventFilter(self, obj, event):
        if obj is self:
            et = event.type()
            # If widget disabled, ignore input entirely
            if not self.isEnabled():
                return True
            if et == QtCore.QEvent.Type.MouseButtonPress:
                if event.button() == QtCore.Qt.MouseButton.LeftButton:
                    self._handle_press(event)
                    return True
            elif et == QtCore.QEvent.Type.MouseMove:
                self._handle_move(event)
                return True
            elif et == QtCore.QEvent.Type.MouseButtonRelease:
                if event.button() == QtCore.Qt.MouseButton.LeftButton:
                    self._handle_release(event)
                    return True
            elif et == QtCore.QEvent.Type.Leave:
                self._handle_release(None)
                return True
        return super().eventFilter(obj, event)

    def _handle_press(self, event: QtGui.QMouseEvent):
        p = event.position()
        if self.mode == "click":
            if callable(self.on_click):
                self.on_click(float(p.x()), float(p.y()), self.width(), self.height())
            self.clear()
            self._draw_dot(p, color="#FF6600", radius=5)
        elif self.mode == "drag":
            if not self._pressed:
                self.clear()
            self._pressed = True
            self._last_pos = p
            self._last_time = time.monotonic()
            self._restart_zero_timer()

    def _handle_move(self, event: QtGui.QMouseEvent):
        # Drag mode no longer used; keep behavior harmlessly
        if self.mode != "drag" or not self._pressed:
            return
        p = event.position()
        now = time.monotonic()
        if self._last_pos is not None and self._last_time is not None:
            self._draw_line(self._last_pos, p, color="#0099FF")
        self._last_pos = p
        self._last_time = now
        self._restart_zero_timer()

    def _handle_release(self, _event: QtGui.QMouseEvent | None):
        if self.mode == "drag":
            self._pressed = False
            self._last_pos = None
            self._last_time = None
            self._stop_zero_timer()
            self._emit_zero_velocity()  # zero on release

    def _restart_zero_timer(self):
        if self.mode == "drag":
            self._zero_timer.start(self._zero_interval_ms)

    def _stop_zero_timer(self):
        if self._zero_timer.isActive():
            self._zero_timer.stop()

    def _emit_zero_velocity(self):
        if callable(self.on_velocity):
            self.on_velocity(0.0, 0.0)


# ---------------- Main Window ----------------


class TaserGUI(QtWidgets.QMainWindow):
    # GUI-thread signal
    _map_pixmap_ready = QtCore.pyqtSignal(QtGui.QPixmap, object)  # object = Workspace

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TASER Control Panels (PyQt6)")

        # ROS node
        rclpy.init(args=None)
        self.node = node.Node("taser_gui_qt6")

        # TF buffer/listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)

        # Start with no workspace; left canvas disabled until map arrives
        self.ws: Workspace | None = None

        # Layout
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        hbox = QtWidgets.QHBoxLayout(central)
        hbox.setSpacing(16)
        hbox.setContentsMargins(12, 12, 12, 12)

        # Left panel (map)
        left_col = QtWidgets.QVBoxLayout()
        left_label = QtWidgets.QLabel("Navigation (click to set pose) — Occupancy Grid")
        left_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.left_canvas = CanvasWidget(
            LEFT_PANEL_SIZE, LEFT_PANEL_SIZE, self.ws, mode="click"
        )
        self.left_canvas.set_bg_pixmap(None)
        self.left_canvas.setEnabled(False)  # disabled until map received
        self.left_canvas.set_waiting_text(
            "Waiting for map on /taser/navigation/occupancy_grid …"
        )
        left_col.addWidget(left_label)
        left_col.addWidget(
            self.left_canvas, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter
        )

        # Right control panel (Manipulation with two buttons)
        right_col = QtWidgets.QVBoxLayout()
        panel_title = QtWidgets.QLabel("Manipulation")
        panel_title.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        panel_title.setStyleSheet("font-weight: 600; font-size: 16px;")
        btn_pick = QtWidgets.QPushButton("Pick")
        btn_reset = QtWidgets.QPushButton("Reset")

        # Optional: make the panel a fixed width column
        right_wrapper = QtWidgets.QWidget()
        right_wrapper.setFixedWidth(CONTROL_WIDTH)
        right_inner = QtWidgets.QVBoxLayout(right_wrapper)
        right_inner.setSpacing(12)
        right_inner.addWidget(panel_title)
        right_inner.addWidget(btn_pick)
        right_inner.addWidget(btn_reset)
        right_inner.addStretch(1)

        right_col.addWidget(right_wrapper)

        # Pack
        hbox.addLayout(left_col, stretch=0)
        hbox.addLayout(right_col, stretch=0)

        # Publishers
        self.nav_pub = self.node.create_publisher(
            Pose2D, "/taser/navigation/target_pose", 10
        )
        self.task_pub = self.node.create_publisher(
            Int32, "/taser/manipulation/task", 10
        )

        # Map subscriber (latched-like QoS)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_sub = self.node.create_subscription(
            OccupancyGrid, MAP_TOPIC, self._on_map, qos
        )

        # Connect GUI-thread signal
        self._map_pixmap_ready.connect(self._apply_map_pixmap)

        # Background ROS spin
        self._ros_thread = threading.Thread(
            target=rclpy.spin, args=(self.node,), daemon=True
        )
        self._ros_thread.start()

        # Hooks
        self.left_canvas.on_click = self._publish_navigation_pose

        # Button actions
        btn_pick.clicked.connect(self._send_pick)
        btn_reset.clicked.connect(self._send_reset)

        # TF overlay timer
        self._tf_timer = QtCore.QTimer(self)
        self._tf_timer.timeout.connect(self._update_robot_overlay)
        self._tf_timer.start(50)

        # Size baseline
        self.setMinimumWidth(self.left_canvas.width() + CONTROL_WIDTH + 12 * 2 + 16)
        self.setFixedHeight(max(HEIGHT, self.left_canvas.height()) + 24)

    # --------- Map handling ---------
    def _on_map(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0:
            return

        # data -> grayscale image
        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        img = np.full((height, width), 127, dtype=np.uint8)  # unknown
        img[data == 0] = 255  # free
        img[data > 0] = 0  # occupied

        # Flip vertically to match y-down rendering
        img = np.flipud(img)

        # ---- FIXED QImage CONSTRUCTION (PyQt6-friendly) ----
        img = np.ascontiguousarray(img, dtype=np.uint8)
        h, w = img.shape
        bytes_per_line = w  # 1 byte per pixel in Grayscale8

        qimg = QtGui.QImage(
            img.tobytes(),
            w,
            h,
            bytes_per_line,
            QtGui.QImage.Format.Format_Grayscale8,
        ).copy()
        qpix = QtGui.QPixmap.fromImage(qimg)
        # ----------------------------------------------------

        # Workspace from map (assumes map axes aligned; yaw≈0)
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        new_ws = Workspace(
            x_min=ox - width * res / 2.0,
            x_max=ox + width * res / 2.0,
            y_min=oy - height * res / 2.0,
            y_max=oy + height * res / 2.0,
        )
        self.ws = new_ws

        self._map_pixmap_ready.emit(qpix, new_ws)

    @QtCore.pyqtSlot(QtGui.QPixmap, object)
    def _apply_map_pixmap(self, qpix: QtGui.QPixmap, new_ws: Workspace):
        # Update workspace and canvases
        self.ws = new_ws
        self.left_canvas.ws = new_ws
        # enable clicking now that we have a map
        self.left_canvas.setEnabled(True)
        self.left_canvas.set_waiting_text(None)
        self.left_canvas.set_bg_pixmap(qpix)

    # --------- Publishers / Actions ---------
    def _publish_navigation_pose(self, x_px: float, y_px: float, fw: int, fh: int):
        if self.ws is None:
            return  # safeguard
        xr, yr = frame_to_robot(x_px, y_px, fw, fh, self.ws)
        msg = Pose2D()
        msg.x = float(xr)
        msg.y = float(yr)
        msg.theta = 0.0
        self.nav_pub.publish(msg)
        self.node.get_logger().info(
            f"Navigation target pose: ({msg.x:.3f}, {msg.y:.3f})"
        )

    def _publish_task(self, value: int, label: str):
        if self.task_pub is None:
            return
        msg = Int32()
        msg.data = int(value)
        self.task_pub.publish(msg)
        self.node.get_logger().info(f"Manipulation task '{label}' -> {value}")

    def _send_pick(self):
        self._publish_task(1, "Pick")

    def _send_reset(self):
        self._publish_task(0, "Reset")

    # --------- TF -> robot overlay ---------
    def _update_robot_overlay(self):
        if self.ws is None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame="World",
                source_frame="base_link",
                time=Time(),  # latest
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        xr = float(tf.transform.translation.x)
        yr = float(tf.transform.translation.y)
        self.left_canvas.set_robot_world(xr, yr)

    # --------- Cleanup ---------
    def closeEvent(self, event: QtGui.QCloseEvent):
        try:
            self._tf_timer.stop()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        try:
            # If spinning in a thread, join it
            pass
        except Exception:
            pass
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = TaserGUI()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
