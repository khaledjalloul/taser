from taser.common.datatypes import VelocityCommand


class DifferentialDriveKinematics:
    """
    A class to handle differential drive kinematics.
    """

    def __init__(self, wheel_base: float, wheel_radius: float):
        self._wheel_base = wheel_base
        self._wheel_radius = wheel_radius

    def step(self, vel: VelocityCommand) -> tuple[float, float]:
        v_l = (vel.v - (self._wheel_base / 2) * vel.w) / self._wheel_radius
        v_r = (vel.v + (self._wheel_base / 2) * vel.w) / self._wheel_radius
        return v_l, v_r

    def inverse_step(self, v_l: float, v_r: float) -> VelocityCommand:
        v = (v_l + v_r) * self._wheel_radius / 2
        w = (v_r - v_l) * self._wheel_radius / self._wheel_base
        return VelocityCommand(v=v, w=w)
