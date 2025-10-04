#!/usr/bin/env python3
# Requires: PyQt6, rclpy, geometry_msgs, nav_msgs, tf2_ros

import sys
import threading
import time
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, Vector3
from nav_msgs.msg import OccupancyGrid
from PyQt6 import QtCore, QtGui, QtWidgets
from rclpy import node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
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
MIDDLE_WIDTH = 300
RIGHT_WIDTH = 300
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
      - 'drag'  : press-move-release -> velocities
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
        self.on_velocity = None

        # Drag state
        self._pressed = False
        self._last_pos = None
        self._last_time = None

        # Auto-zero velocity when idle
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
        if self.mode != "drag" or not self._pressed:
            return
        p = event.position()
        now = time.monotonic()
        if self._last_pos is not None and self._last_time is not None:
            self._draw_line(self._last_pos, p, color="#0099FF")
            dx = float(p.x() - self._last_pos.x())
            dy = float(p.y() - self._last_pos.y())
            dt = now - self._last_time

            if dt <= 0.0:
                vx, vy = 0.0, 0.0
            else:
                vx = dx / dt
                vy = -dy / dt
            if dt > 0 and callable(self.on_velocity):
                self.on_velocity(vx, vy)
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

        # Middle panel (unchanged)
        mid_col = QtWidgets.QVBoxLayout()
        mid_label = QtWidgets.QLabel("Arm 1 (press & move: path + velocity)")
        mid_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        dummy_ws = Workspace(0.0, 1.0, 0.0, 1.0)
        self.mid_canvas = CanvasWidget(MIDDLE_WIDTH, HEIGHT, dummy_ws, mode="drag")
        mid_col.addWidget(mid_label)
        mid_col.addWidget(
            self.mid_canvas, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter
        )

        # Right panel (unchanged)
        right_col = QtWidgets.QVBoxLayout()
        right_label = QtWidgets.QLabel("Arm 2 (press & move: path + velocity)")
        right_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.right_canvas = CanvasWidget(RIGHT_WIDTH, HEIGHT, dummy_ws, mode="drag")
        right_col.addWidget(right_label)
        right_col.addWidget(
            self.right_canvas, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter
        )

        # Pack
        hbox.addLayout(left_col, stretch=0)
        hbox.addLayout(mid_col, stretch=1)
        hbox.addLayout(right_col, stretch=1)

        # Publishers
        self.nav_pub = self.node.create_publisher(
            Pose2D, "/taser/navigation/target_pose", 10
        )
        self.arm1_pub = self.node.create_publisher(
            Vector3, "/taser/manipulation/left_arm_target_velocity", 10
        )
        self.arm2_pub = self.node.create_publisher(
            Vector3, "/taser/manipulation/right_arm_target_velocity", 10
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
        self.mid_canvas.on_velocity = self._publish_arm1_vel
        self.right_canvas.on_velocity = self._publish_arm2_vel

        # TF overlay timer
        self._tf_timer = QtCore.QTimer(self)
        self._tf_timer.timeout.connect(self._update_robot_overlay)
        self._tf_timer.start(50)

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
        # Ensure C-contiguous bytes and give bytesPerLine explicitly.
        img = np.ascontiguousarray(img, dtype=np.uint8)
        h, w = img.shape
        bytes_per_line = w  # 1 byte per pixel in Grayscale8

        qimg = QtGui.QImage(
            img.tobytes(),  # bytes buffer (not memoryview)
            w,
            h,
            bytes_per_line,
            QtGui.QImage.Format.Format_Grayscale8,
        ).copy()  # copy so QImage owns the memory
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

    # --------- Publishers ---------
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

    def _publish_arm1_vel(self, vx: float, vy: float):
        vx = vx * 0.5 / self.mid_canvas.width()
        vy = vy * 1.0 / self.mid_canvas.height()
        self.arm1_pub.publish(Vector3(x=float(vx), y=0.0, z=float(vy)))

    def _publish_arm2_vel(self, vx: float, vy: float):
        vx = vx * 0.5 / self.right_canvas.width()
        vy = vy * 1.0 / self.right_canvas.height()
        self.arm2_pub.publish(Vector3(x=float(vx), y=0.0, z=float(vy)))

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
            if self._ros_thread.is_alive():
                self._ros_thread.join(timeout=1.0)
        except Exception:
            pass
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = TaserGUI()
    win.setMinimumWidth(
        win.left_canvas.width()
        + win.mid_canvas.width()
        + win.right_canvas.width()
        + 16 * 2
        + 60
    )
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
