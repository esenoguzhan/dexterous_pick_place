"""Check the actual object-to-flange convention and malformed input handling."""

import math
import unittest

from lara5_object_approach.geometry import object_to_flange


class GeometryTest(unittest.TestCase):
    def test_identity_yaw_stops_above_object_centre(self):
        position, q = object_to_flange((0.2, -0.3, 0.95), (0, 0, 0, 1),
                                       (0, 0, 0.2), (-math.pi/2, math.pi/2, 0))
        for actual, expected in zip(position, (0.2, -0.3, 1.15)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(q, (-0.5, 0.5, 0.5, 0.5)):
            self.assertAlmostEqual(actual, expected)

    def test_object_yaw_rotates_offset_and_hand_together(self):
        s = math.sqrt(0.5)
        position, q = object_to_flange((1, 2, 3), (0, 0, s, s),
                                       (0.1, 0.2, 0.4), (math.pi, 0, 0))
        for actual, expected in zip(position, (0.8, 2.1, 3.4)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(q, (s, s, 0, 0)):
            self.assertAlmostEqual(actual, expected)

    def test_quaternion_sign_does_not_change_physical_target(self):
        args = ((1, 2, 3), (0, 0, 0, 1), (0.1, 0, 0.4), (0, 0, math.pi/2))
        p, q = object_to_flange(*args)
        other_p, other_q = object_to_flange(args[0], (0, 0, 0, -1), *args[2:])
        self.assertEqual(p, other_p)
        for a, b in zip(q, other_q):
            self.assertAlmostEqual(a, -b)

    def test_invalid_inputs_are_rejected(self):
        valid = [(0, 0, 1), (0, 0, 0, 1), (0, 0, 0.4), (math.pi, 0, 0)]
        for index, bad in ((0, (math.nan, 0, 1)), (0, (0, 1)),
                           (1, (0, 0, 0, 0)), (1, (0, 0, 0, 2)),
                           (1, (0, 0, math.inf, 1)),
                           (1, (math.sqrt(0.5), 0, 0, math.sqrt(0.5))),
                           (2, (0, 0, math.inf)), (3, (0, math.nan, 0))):
            with self.subTest(index=index, bad=bad):
                args = list(valid)
                args[index] = bad
                with self.assertRaises(ValueError):
                    object_to_flange(*args)


if __name__ == "__main__":
    unittest.main()
