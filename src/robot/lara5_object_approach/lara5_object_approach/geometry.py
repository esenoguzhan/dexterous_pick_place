"""Object-centre to flange transform, without ROS or camera dependencies.

Quaternions use ROS order (x, y, z, w). Perception currently supplies yaw only.
"""

import math


def finite_vector(values, length, name):
    values = tuple(float(value) for value in values)
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return values


def quaternion_from_rpy(rpy):
    roll, pitch, yaw = finite_vector(rpy, 3, "object_to_flange_rpy")
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def object_to_flange(position, orientation, offset, rpy):
    """Compose T_world_object * T_object_flange for an upright table object."""
    px, py, pz = finite_vector(position, 3, "object position")
    q = finite_vector(orientation, 4, "object quaternion")
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or abs(norm - 1.0) > 0.01:
        raise ValueError("object orientation must be a unit quaternion")
    x, y, z, w = (value / norm for value in q)
    if math.hypot(x, y) > 1e-3:
        raise ValueError("expected a yaw-only object orientation in world")
    dx, dy, dz = finite_vector(offset, 3, "object_to_flange_xyz")
    # Rotate the offset with the full normalized quaternion to compose exactly.
    target_position = (
        px + (1 - 2 * (y*y + z*z))*dx + 2*(x*y - z*w)*dy + 2*(x*z + y*w)*dz,
        py + 2*(x*y + z*w)*dx + (1 - 2*(x*x + z*z))*dy + 2*(y*z - x*w)*dz,
        pz + 2*(x*z - y*w)*dx + 2*(y*z + x*w)*dy + (1 - 2*(x*x + y*y))*dz,
    )
    a, b, c, d = quaternion_from_rpy(rpy)
    target_orientation = (w*a + x*d + y*c - z*b, w*b - x*c + y*d + z*a,
                          w*c + x*b - y*a + z*d, w*d - x*a - y*b - z*c)
    return (finite_vector(target_position, 3, "flange position"),
            finite_vector(target_orientation, 4, "flange quaternion"))
