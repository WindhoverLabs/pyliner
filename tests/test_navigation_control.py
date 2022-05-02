import unittest

from pyliner.apps.navigation.control import pi


class TestControl(unittest.TestCase):
    def test_pi(self):
        p = pi(0.200, 0.4935, 1.0, 1.0)
        x = 0.0
        for _ in range(100):
            x += p(x, 0.0)
            x -= 1.0
        self.assertAlmostEqual(0.0, x)
