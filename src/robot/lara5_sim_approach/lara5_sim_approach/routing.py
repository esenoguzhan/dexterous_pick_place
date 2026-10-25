"""Bounded Cartesian waypoint routing; every candidate needs native IK validation.

This module proposes geometry only. It never labels a route collision-free:
the caller must solve and check the entire arm for every segment and approach.
"""

from dataclasses import dataclass
import math
import time


class RouteRejected(RuntimeError):
    """A candidate failed IK, continuity, limits or native collision checking."""


@dataclass(frozen=True)
class Route:
    name: str
    waypoints: tuple


def waypoint_routes(start, goal, lift_step=0.08, detour_margin=0.12):
    start, goal = tuple(start), tuple(goal)
    if (len(start) != 3 or len(goal) != 3 or
            not all(math.isfinite(value) for value in (*start, *goal, lift_step, detour_margin)) or
            lift_step <= 0 or detour_margin <= 0):
        raise ValueError("Finite XYZ positions and positive routing distances are required")
    routes = [Route("direct", (goal,))]
    for index in (1, 2):
        z = max(start[2], goal[2]) + index * lift_step
        routes.append(Route(f"lift_{index}", ((start[0], start[1], z), (goal[0], goal[1], z), goal)))
    z = max(start[2], goal[2]) + lift_step
    for axis in (0, 1):
        for sign in (-1, 1):
            boundary = min(start[axis], goal[axis]) if sign < 0 else max(start[axis], goal[axis])
            side = boundary + sign * detour_margin
            a, b = list(start), list(goal)
            a[axis] = b[axis] = side
            a[2] = b[2] = z
            routes.append(Route(f"side_{'xy'[axis]}_{sign:+d}", (
                (start[0], start[1], z), tuple(a), tuple(b), (goal[0], goal[1], z), goal)))

    def length(route):
        points = (start, *route.waypoints)
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))

    return [routes[0], *sorted(routes[1:], key=length)]


def find_route(routes, orientations, attempt, max_attempts=28, timeout=30.0,
               on_reject=None, clock=time.monotonic):
    """Return the first fully validated route; never dispatch or hide service errors."""
    if (not isinstance(max_attempts, int) or isinstance(max_attempts, bool) or max_attempts <= 0 or
            not math.isfinite(timeout) or timeout <= 0):
        raise ValueError("Positive integer route_attempts and finite positive planning_timeout required")
    deadline, count, last_error = clock() + timeout, 0, "No candidates"
    for route in routes:
        for orientation in orientations:
            if count >= max_attempts or clock() >= deadline:
                raise RouteRejected(f"NEURA IK routing failed after {count} attempts: {last_error}")
            count += 1
            try:
                return route, attempt(route, orientation), count
            except RouteRejected as error:
                last_error = str(error)
                if on_reject is not None:
                    on_reject(route.name, last_error)
    raise RouteRejected(f"NEURA IK routing failed after {count} attempts: {last_error}")
