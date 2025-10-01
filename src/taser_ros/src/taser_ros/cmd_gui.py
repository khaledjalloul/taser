#!/usr/bin/env python3
# Requires: PyQt6, rclpy, geometry_msgs, tf2_ros

import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Pose2D, Vector3
from PyQt6 import QtCore, QtGui, QtWidgets
from rclpy import node
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from taser.common.datatypes import Polygon, Workspace
from taser_ros.parameters import load_sim_parameters

# ---------------- Parameters & Mapping ----------------

# Ensure stdout is line-buffered (for logging in terminals/launch)
sys.stdout.reconfigure(line_buffering=True)


def load_gui_parameters(node: node.Node) -> dict:
    sim_params = load_sim_parameters(node)
    return {
        "left_panel_size": 600,  # square (width = height)
        "middle_width": 300,
        "right_width": 300,
        "height": 600,
        "workspace": sim_params.navigation.workspace,
        "polygons": sim_params.navigation.polygons,
    }


def frame_to_robot(x_px: float, y_px: float, fw: int, fh: int, ws: Workspace):
    """Map frame (pixels) -> robot (meters), y-up in robot."""
    xr = (x_px / fw) * (ws.x_max - ws.x_min) + ws.x_min
    yr = ((fh - y_px) / fh) * (ws.y_max - ws.y_min) + ws.y_min
    return xr, yr


def robot_to_frame(xr: float, yr: float, fw: int, fh: int, ws: Workspace):
    """Robot (meters, y-up) -> frame (pixels, y-down)."""
    x_px = (xr - ws.x_min) / (ws.x_max - ws.x_min) * fw
    y_px = fh - ((yr - ws.y_min) / (ws.y_max - ws.y_min) * fh)
    return x_px, y_px


# ---------------- Canvas ----------------


class CanvasWidget(QtWidgets.QWidget):
    """
    Canvas with three layers:
      - Background: static polygons
      - Robot overlay: current robot circle (updated via setter)
      - Foreground: interactive dot/path

    Modes:
      - 'click': single click publishes pose; shows one dot (clears previous)
      - 'drag' : press-move-release draws a path; publishes velocity; auto-zeros when idle
    """

    def __init__(self, width: int, height: int, ws: Workspace, mode: str, parent=None):
        super().__init__(parent)
        self.setFixedSize(width, height)

        # Layers
        self._bg_pixmap = QtGui.QPixmap(width, height)
        self._fg_pixmap = QtGui.QPixmap(width, height)
        self._bg_pixmap.fill(QtGui.QColor("white"))
        self._fg_pixmap.fill(QtGui.QColor(0, 0, 0, 0))  # transparent

        # Robot overlay state (drawn in paintEvent)
        self._robot_px: QtCore.QPointF | None = None
        self._robot_radius_px: float = 6.0
        self._robot_color = QtGui.QColor("#D81B60")  # pink-ish
        self._robot_edge = QtGui.QColor("#880E4F")

        self.setMouseTracking(True)
        self.setCursor(QtGui.QCursor(QtCore.Qt.CursorShape.CrossCursor))

        self.mode = mode
        self.ws = ws
        self.on_click = None
        self.on_velocity = None

        # Drag state
        self._pressed = False
        self._last_pos = None  # QPointF
        self._last_time = None  # float (monotonic)

        # Auto-zero velocity when idle
        self._zero_timer = QtCore.QTimer(self)
        self._zero_timer.setSingleShot(True)
        self._zero_timer.timeout.connect(self._emit_zero_velocity)
        self._zero_interval_ms = 120

        self.installEventFilter(self)

    # -------- Background (polygons) --------
    def set_polygons_world(
        self, polygons: list[Polygon], color_fill="#E8F0FE", color_edge="#5F6368"
    ):
        self._bg_pixmap.fill(QtGui.QColor("white"))
        painter = QtGui.QPainter(self._bg_pixmap)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        fill = QtGui.QBrush(QtGui.QColor(color_fill))
        edge = QtGui.QPen(QtGui.QColor(color_edge))
        edge.setWidth(1)

        for poly in polygons or []:
            pts_world = [self._xy_of(p) for p in poly]
            if not pts_world:
                continue
            if len(pts_world) == 2:
                # Axis-aligned rectangle from diagonal corners
                (x1, y1), (x2, y2) = pts_world
                xmin, xmax = sorted([x1, x2])
                ymin, ymax = sorted([y1, y2])
                x0_px, y0_px = robot_to_frame(
                    xmin, ymax, self.width(), self.height(), self.ws
                )
                x1_px, y1_px = robot_to_frame(
                    xmax, ymin, self.width(), self.height(), self.ws
                )
                rect = QtCore.QRectF(x0_px, y0_px, x1_px - x0_px, y1_px - y0_px)
                painter.setPen(edge)
                painter.setBrush(fill)
                painter.drawRect(rect)
            else:
                qpoints = []
                for xr, yr in pts_world:
                    x_px, y_px = robot_to_frame(
                        xr, yr, self.width(), self.height(), self.ws
                    )
                    qpoints.append(QtCore.QPointF(x_px, y_px))
                polygonf = QtGui.QPolygonF(qpoints)
                painter.setPen(edge)
                painter.setBrush(fill)
                painter.drawPolygon(polygonf)

        painter.end()
        self.update()

    @staticmethod
    def _xy_of(p):
        if hasattr(p, "x") and hasattr(p, "y"):
            return float(p.x), float(p.y)
        if isinstance(p, (tuple, list)) and len(p) >= 2:
            return float(p[0]), float(p[1])
        raise ValueError(f"Invalid polygon point: {p}")

    # -------- Robot overlay --------
    def set_robot_world(self, xr: float, yr: float):
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

        # Draw robot overlay (circle) between bg and fg
        if self._robot_px is not None:
            painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
            painter.setBrush(QtGui.QBrush(self._robot_color))
            pen = QtGui.QPen(self._robot_edge)
            pen.setWidth(2)
            painter.setPen(pen)
            painter.drawEllipse(
                self._robot_px, self._robot_radius_px, self._robot_radius_px
            )

        painter.drawPixmap(0, 0, self._fg_pixmap)

    # ---- Events ----
    def eventFilter(self, obj, event):
        if obj is self:
            et = event.type()
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
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TASER Control Panels (PyQt6)")

        # ROS node
        rclpy.init(args=None)
        self.node = node.Node("taser_gui_qt6")

        # TF buffer/listener (reuse node’s executor thread)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)

        # Parameters
        self.params = load_gui_parameters(self.node)
        self.ws: Workspace = self.params["workspace"]
        self.polygons = self.params.get("polygons", [])

        # Sizes
        left_size = int(self.params["left_panel_size"])
        height = int(self.params["height"])
        mid_w = int(self.params["middle_width"])
        right_w = int(self.params["right_width"])

        # Layout
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        hbox = QtWidgets.QHBoxLayout(central)
        hbox.setSpacing(16)
        hbox.setContentsMargins(12, 12, 12, 12)

        # Left panel
        left_col = QtWidgets.QVBoxLayout()
        left_label = QtWidgets.QLabel("Navigation (click to set pose)")
        left_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.left_canvas = CanvasWidget(left_size, left_size, self.ws, mode="click")
        self.left_canvas.set_polygons_world(
            self.polygons, color_fill="#E8F0FE", color_edge="#5F6368"
        )
        left_col.addWidget(left_label)
        left_col.addWidget(
            self.left_canvas, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter
        )

        # Middle panel
        mid_col = QtWidgets.QVBoxLayout()
        mid_label = QtWidgets.QLabel("Arm 1 (press & move: path + velocity)")
        mid_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.mid_canvas = CanvasWidget(mid_w, height, self.ws, mode="drag")
        mid_col.addWidget(mid_label)
        mid_col.addWidget(
            self.mid_canvas, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter
        )

        # Right panel
        right_col = QtWidgets.QVBoxLayout()
        right_label = QtWidgets.QLabel("Arm 2 (press & move: path + velocity)")
        right_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignHCenter)
        self.right_canvas = CanvasWidget(right_w, height, self.ws, mode="drag")
        right_col.addWidget(right_label)
        right_col.addWidget(
            self.right_canvas, alignment=QtCore.Qt.AlignmentFlag.AlignHCenter
        )

        # Pack into HBox
        hbox.addLayout(left_col, stretch=0)
        hbox.addLayout(mid_col, stretch=1)
        hbox.addLayout(right_col, stretch=1)

        # ROS publishers
        self.nav_pub = self.node.create_publisher(
            Pose2D, "/taser/navigation_target_pose", 10
        )
        self.arm1_pub = self.node.create_publisher(
            Vector3, "/taser/arm_1_target_velocity", 10
        )
        self.arm2_pub = self.node.create_publisher(
            Vector3, "/taser/arm_2_target_velocity", 10
        )

        # Background ROS spin (so TF buffer gets data)
        self._ros_thread = threading.Thread(
            target=rclpy.spin, args=(self.node,), daemon=True
        )
        self._ros_thread.start()

        # Hook up callbacks
        self.left_canvas.on_click = self._publish_navigation_pose
        self.mid_canvas.on_velocity = self._publish_arm1_vel
        self.right_canvas.on_velocity = self._publish_arm2_vel

        # Timer to refresh robot pose overlay (20 Hz)
        self._tf_timer = QtCore.QTimer(self)
        self._tf_timer.timeout.connect(self._update_robot_overlay)
        self._tf_timer.start(50)

    # --------- Publishers ---------
    def _publish_navigation_pose(self, x_px: float, y_px: float, fw: int, fh: int):
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
        vx = vx * 0.5 / self.mid_canvas.width()  # m per pixel in x
        vy = vy * 1.0 / self.mid_canvas.height()  # m per pixel in y
        print("arm 1", vy, self.mid_canvas.height())
        self.arm1_pub.publish(Vector3(x=float(vx), y=0.0, z=float(vy)))

    def _publish_arm2_vel(self, vx: float, vy: float):
        vx = vx * 0.5 / self.right_canvas.width()  # m per pixel in x
        vy = vy * 1.0 / self.right_canvas.height()  # m per pixel in y
        print("arm 2", vy, self.right_canvas.height())
        self.arm2_pub.publish(Vector3(x=float(vx), y=0.0, z=float(vy)))

    # --------- TF -> robot overlay ---------
    def _update_robot_overlay(self):
        """Look up base_link in map and update the robot circle on the left canvas."""
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame="map",
                source_frame="base_link",
                time=Time(),  # latest
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            # No pose yet; skip quietly or log at low rate if desired
            # self.node.get_logger().debug(f"TF lookup failed: {e}")
            return

        xr = float(tf.transform.translation.x)
        yr = float(tf.transform.translation.y)
        # theta (not used for the circle, but handy if you want an arrow):
        # theta = 2.0 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w)

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
