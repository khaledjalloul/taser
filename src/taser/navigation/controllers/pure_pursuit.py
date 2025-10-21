import math
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt

from taser.common.datatypes import Pose, Vec2, VelocityCommand


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class PurePursuitController:
    """
    Minimal pure pursuit for diff-drive.
    - Path: list of (x,y) waypoints (meters), in world frame.
    - step(pose, v_curr) -> (VelocityCommand, reached, info)
    """

    def __init__(
        self,
        lookahead_base: float = 0.25,  # L0 (m)
        lookahead_gain: float = 0.5,  # kv (s) -> Ld = L0 + kv*|v|
        min_lookahead: float = 0.15,
        max_lookahead: float = 1.00,
        v_max: float = 0.6,
        w_max: float = 1.2,
        curve_slowdown: float = 1.5,  # alpha in v = v_max/(1+alpha|kappa|)
        heading_turn_thresh: float = math.radians(50.0),  # when target far behind
        goal_pos_tol: float = 0.08,  # m
        goal_yaw_tol: float = math.radians(5.0),  # rad
        turn_gain: float = 2.0,
    ):  # P-gain for in-place turns
        self.L0 = lookahead_base
        self.kv = lookahead_gain
        self.Lmin = min_lookahead
        self.Lmax = max_lookahead
        self.v_max = v_max
        self.w_max = w_max
        self.alpha = curve_slowdown
        self.heading_turn_thresh = heading_turn_thresh
        self.goal_pos_tol = goal_pos_tol
        self.goal_yaw_tol = goal_yaw_tol
        self.turn_gain = turn_gain

        self._path: List[Pose] = []
        self._cum_s: List[float] = [0.0]
        self._total_s: float = 0.0
        self._goal_yaw: Optional[float] = None

    # ---------- public API ----------
    def set_path(self, pts: List[Pose], goal_yaw: Optional[float] = None):
        if len(pts) < 2:
            raise ValueError("Path must have at least 2 points.")
        self._path = pts
        self._goal_yaw = goal_yaw
        self._precompute_lengths()

    def step(
        self,
        pose: Pose,
        v_current: float = 0.0,
        obstacle_distance_ahead: Optional[float] = None,
        safety_radius: float = 0.25,
        slow_zone: float = 0.5,
    ) -> Tuple[VelocityCommand, bool, Dict]:
        """
        obstacle_distance_ahead: if provided (meters), we slow as it approaches safety_radius.
        safety_radius: robot radius + margin
        slow_zone: distance span over which we ramp speed (m)
        """
        if self._path is None or len(self._path) < 2:
            return VelocityCommand(0.0, 0.0), True, {"reason": "no_path"}

        # Goal check (position)
        gx, gy = self._path[-1].x, self._path[-1].y
        dxg, dyg = gx - pose.x, gy - pose.y
        dist_goal = math.hypot(dxg, dyg)
        if dist_goal <= self.goal_pos_tol:
            # Align heading if requested
            if self._goal_yaw is not None:
                yaw_err = wrap_angle(self._goal_yaw - pose.rz)
                if abs(yaw_err) > self.goal_yaw_tol:
                    w = max(-self.w_max, min(self.w_max, self.turn_gain * yaw_err))
                    return (
                        VelocityCommand(0.0, w),
                        False,
                        {"reason": "final_yaw_align"},
                    )
            return VelocityCommand(0.0, 0.0), True, {"reason": "goal_reached"}

        # 1) Closest point on path → arc-length s_closest
        s_closest, closest_xy = self._closest_s_on_path((pose.x, pose.y))

        # 2) Lookahead target at s = s_closest + Ld
        Ld = max(self.Lmin, min(self.L0 + self.kv * abs(v_current), self.Lmax))
        s_target = min(self._total_s, s_closest + Ld)
        tx, ty = self._interpolate_at_s(s_target)

        # 3) Transform target to robot frame
        ct, st = math.cos(pose.rz), math.sin(pose.rz)
        dx, dy = tx - pose.x, ty - pose.y
        xr = ct * dx + st * dy
        yr = -st * dx + ct * dy

        # 4) Heading to target
        ang_to_tgt = math.atan2(yr, xr)

        # 5) Curvature & nominal speed
        kappa = 0.0 if Ld < 1e-6 else (2.0 * yr) / (Ld * Ld)
        v_cmd = self.v_max / (1.0 + self.alpha * abs(kappa))

        # 6) Obstacle-based slowdown (optional)
        if obstacle_distance_ahead is not None:
            # Ramp from safety_radius → safety_radius+slow_zone
            m = (obstacle_distance_ahead - safety_radius) / max(1e-6, slow_zone)
            m = max(0.0, min(1.0, m))
            v_cmd *= m

        # 7) Decide if we should rotate in place (target behind / big heading error)
        rotate_in_place = (xr < 0.0 and abs(ang_to_tgt) > math.radians(20)) or (
            abs(ang_to_tgt) > self.heading_turn_thresh
        )
        if rotate_in_place:
            w = max(-self.w_max, min(self.w_max, self.turn_gain * ang_to_tgt))
            return (
                VelocityCommand(0.0, w),
                False,
                {
                    "reason": "rotate_in_place",
                    "ang_to_tgt_deg": math.degrees(ang_to_tgt),
                },
            )

        # 8) Curvature → omega, clamp limits
        w_cmd = kappa * v_cmd
        w_cmd = max(-self.w_max, min(self.w_max, w_cmd))
        v_cmd = max(
            0.0, min(self.v_max, v_cmd)
        )  # forward-only; allow neg if you want reversing

        return (
            VelocityCommand(v_cmd, w_cmd),
            False,
            {
                "Ld": Ld,
                "kappa": kappa,
                "ang_to_tgt_deg": math.degrees(ang_to_tgt),
                "s_closest": s_closest,
                "s_target": s_target,
                "target": (tx, ty),
            },
        )

    # ---------- internals ----------
    def _precompute_lengths(self):
        self._cum_s = [0.0]
        for i in range(len(self._path) - 1):
            x1, y1 = self._path[i].x, self._path[i].y
            x2, y2 = self._path[i + 1].x, self._path[i + 1].y
            self._cum_s.append(self._cum_s[-1] + math.hypot(x2 - x1, y2 - y1))
        self._total_s = self._cum_s[-1]

    def _interpolate_at_s(self, s: float) -> Vec2:
        # s in [0, total]; find segment i with cum_s[i] <= s <= cum_s[i+1]
        if s <= 0.0:
            return self._path[0].x, self._path[0].y
        if s >= self._total_s:
            return self._path[-1].x, self._path[-1].y
        # linear search is fine for modest paths; for long paths, binary search would be faster
        i = 0
        while i + 1 < len(self._cum_s) and self._cum_s[i + 1] < s:
            i += 1
        seg_len = self._cum_s[i + 1] - self._cum_s[i]
        t = 0.0 if seg_len < 1e-9 else (s - self._cum_s[i]) / seg_len
        x1, y1 = self._path[i].x, self._path[i].y
        x2, y2 = self._path[i + 1].x, self._path[i + 1].y
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))

    def _closest_s_on_path(self, p: Vec2) -> Tuple[float, Vec2]:
        """Return (s_closest, closest_point)."""
        px, py = p
        best_d2 = float("inf")
        best_s = 0.0
        best_pt = self._path[0]
        for i in range(len(self._path) - 1):
            x1, y1 = self._path[i].x, self._path[i].y
            x2, y2 = self._path[i + 1].x, self._path[i + 1].y
            vx, vy = x2 - x1, y2 - y1
            seg2 = vx * vx + vy * vy
            if seg2 < 1e-12:
                # degenerate
                d2 = (px - x1) ** 2 + (py - y1) ** 2
                if d2 < best_d2:
                    best_d2, best_s, best_pt = d2, self._cum_s[i], (x1, y1)
                continue
            # projection
            t = ((px - x1) * vx + (py - y1) * vy) / seg2
            t = max(0.0, min(1.0, t))
            cx, cy = x1 + t * vx, y1 + t * vy
            d2 = (px - cx) ** 2 + (py - cy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_s = self._cum_s[i] + math.sqrt(seg2) * t
                best_pt = (cx, cy)
        return best_s, best_pt
