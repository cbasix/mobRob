
import unittest

from vector import Vector2D
from math import pi as PI


class VectorAngleCase(unittest.TestCase):
    def test_create_by(self):
        t = Vector2D.create_by(5, PI)
        self.assertAlmostEqual(PI, t.get_angle())

    def test_get_angle(self):
        t = Vector2D(1, 0)
        self.assertAlmostEqual(0, t.get_angle())

        t = Vector2D(1, 1)
        self.assertAlmostEqual(PI / 4, t.get_angle())

        t = Vector2D(0, 1)
        self.assertAlmostEqual(PI / 2, t.get_angle())

        t = Vector2D(-1, 1)
        self.assertAlmostEqual(PI * (3/4), t.get_angle())

        t = Vector2D(-1, -1)
        self.assertAlmostEqual(-PI * (3 / 4), t.get_angle())

        t = Vector2D(1, -1)
        self.assertAlmostEqual(-PI / 4, t.get_angle())