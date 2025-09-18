from taser.common.datatypes import VelocityCommand


def get_wheel_velocities(
    vel: VelocityCommand, L: float, wheel_radius: float
) -> tuple[float, float]:
    """
    Convert linear and angular velocity to left and right wheel velocities.

    Args:
        vel: VelocityCommand containing linear (v) and angular (w) velocities.
        L: Distance between the wheels (wheelbase).
        wheel_radius: Radius of the wheels.
    Returns:
        A tuple containing left and right wheel velocities (v_l, v_r).
    """
    v_l = (vel.v - (L / 2) * vel.w) / wheel_radius
    v_r = (vel.v + (L / 2) * vel.w) / wheel_radius
    return v_l, v_r


def get_base_velocity(
    v_l: float, v_r: float, L: float, wheel_radius: float
) -> VelocityCommand:
    """
    Convert left and right wheel velocities to linear and angular velocity.

    Args:
        v_l: Left wheel velocity.
        v_r: Right wheel velocity.
        L: Distance between the wheels (wheelbase).
        wheel_radius: Radius of the wheels.
    Returns:
        A VelocityCommand containing linear (v) and angular (w) velocities.
    """
    v = (v_l + v_r) * wheel_radius / 2
    w = (v_r - v_l) * wheel_radius / L
    return VelocityCommand(v=v, w=w)
