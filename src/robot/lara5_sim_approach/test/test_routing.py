"""Native route-policy tests with injected IK outcomes; no ROS or simulation."""

import math
import unittest

from lara5_sim_approach.routing import RouteRejected, find_route, waypoint_routes


class RoutingTests(unittest.TestCase):
    def setUp(self):
        self.start, self.goal = (0., 0., 1.3), (0.2, 0.1, 1.4)
        self.routes = waypoint_routes(self.start, self.goal)

    def test_direct_first_and_detours_return_to_exact_goal(self):
        self.assertEqual(self.routes[0].name, "direct")
        self.assertEqual(len(self.routes), 7)
        for route in self.routes:
            self.assertEqual(route.waypoints[-1], self.goal)
            self.assertTrue(all(math.isfinite(x) for point in route.waypoints for x in point))
        for route in self.routes[1:]:
            self.assertGreater(route.waypoints[0][2], max(self.start[2], self.goal[2]))

    def test_blocked_direct_tries_other_yaws_then_detours(self):
        calls = []

        def native_ik(route, yaw):
            calls.append((route.name, yaw))
            if route.name == "direct":
                raise RouteRejected("too close: 29.7937 mm, minimum 30 mm")
            return "checked native path and checked approach"

        route, value, count = find_route(self.routes, [0, 90], native_ik)
        self.assertNotEqual(route.name, "direct")
        self.assertEqual(calls[:2], [("direct", 0), ("direct", 90)])
        self.assertEqual(count, 3)
        self.assertEqual(value, "checked native path and checked approach")

    def test_approach_failure_rejects_complete_candidate(self):
        def native_ik(route, yaw):
            if yaw == 0:
                raise RouteRejected("final approach blocked")
            return "both phases checked"

        route, value, count = find_route(self.routes, [0, 90], native_ik)
        self.assertEqual(route.name, "direct")
        self.assertEqual(count, 2)
        self.assertEqual(value, "both phases checked")

    def test_all_blocked_stops_at_attempt_limit(self):
        calls = []

        def blocked(route, yaw):
            calls.append(route.name)
            raise RouteRejected("collision")

        with self.assertRaisesRegex(RouteRejected, "failed after 3 attempts: collision"):
            find_route(self.routes, [0, 90], blocked, max_attempts=3)
        self.assertEqual(len(calls), 3)

    def test_timeout_and_service_failures_do_not_fall_through_to_execution(self):
        clock = [0.]

        def blocked(route, yaw):
            clock[0] = 2.
            raise RouteRejected("blocked")

        with self.assertRaisesRegex(RouteRejected, "failed after 1 attempts"):
            find_route(self.routes, [0, 90], blocked, timeout=1., clock=lambda: clock[0])

        def disconnected(route, yaw):
            raise RuntimeError("Service unavailable")

        with self.assertRaisesRegex(RuntimeError, "Service unavailable"):
            find_route(self.routes, [0, 90], disconnected)

    def test_invalid_parameters(self):
        with self.assertRaises(ValueError):
            waypoint_routes(self.start, [math.nan, 0, 1])
        with self.assertRaises(ValueError):
            waypoint_routes(self.start, self.goal, detour_margin=-0.1)
        for count in (0, True, 2.5):
            with self.assertRaises(ValueError):
                find_route(self.routes, [0], lambda *args: None, max_attempts=count)


if __name__ == "__main__":
    unittest.main()
