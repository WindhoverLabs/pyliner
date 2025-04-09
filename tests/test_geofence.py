import unittest

from pyliner.apps.geofence import Geofence, LayerKind
from pyliner.apps.geofence.volume import Box, VerticalCylinder
from pyliner.apps.geographic_app import GeographicApp
from pyliner.position import Position, Coordinate


class TestGeofence(unittest.TestCase):
    def test_geofence(self):
        geo = GeographicApp()
        fence = Geofence()

        base = fence.add_layer(0, 'base', LayerKind.ADDITIVE)
        base.add(Box(Position(1, 3, 0), Position(3, 5, 1000)))

        tfr = fence.add_layer(1, 'tfr', LayerKind.SUBTRACTIVE)
        tfr.add(VerticalCylinder(geo, Coordinate(2, 4), 500, 0, 1000))

        self.assertIn(Position(1.1, 3.2, 250), fence)
        self.assertNotIn(Position(2, 4, 100), fence)
        self.assertNotIn(Position(0, 0, 0), fence)
        self.assertNotIn(Position(1.1, 3.2, 11000), fence)


class TestBox(unittest.TestCase):
    def test_box(self):
        box = Box(Position(-1, -1, -1), Position(1, 1, 1))

        # Check 8 corners and center
        self.assertIn(Position(-1, -1, -1), box)
        self.assertIn(Position(-1, -1, 1), box)
        self.assertIn(Position(-1, 1, -1), box)
        self.assertIn(Position(-1, 1, 1), box)
        self.assertIn(Position(1, -1, -1), box)
        self.assertIn(Position(1, -1, 1), box)
        self.assertIn(Position(1, 1, -1), box)
        self.assertIn(Position(1, 1, 1), box)
        self.assertIn(Position(0, 0, 0), box)
        
        # Check outside in each axis
        self.assertNotIn(Position(0, 0, -2), box)
        self.assertNotIn(Position(0, 0, 2), box)
        self.assertNotIn(Position(0, -2, 0), box)
        self.assertNotIn(Position(0, 2, 0), box)
        self.assertNotIn(Position(-2, 0, 0), box)
        self.assertNotIn(Position(2, 0, 0), box)


class TestComposite(unittest.TestCase):
    def test_composite(self):
        box1 = Box(Position(0, 0, 0), Position(2, 2, 2))
        box2 = Box(Position(4, 4, 4), Position(6, 6, 6))

        union = box1 | box2

        # Check in each box
        self.assertIn(Position(1, 1, 1), union)
        self.assertIn(Position(5, 5, 5), union)

        # Check not in between but in bounding box
        self.assertNotIn(Position(3, 3, 3), union)
        self.assertIn(Position(3, 3, 3), union.bounding_box)
        

class TestCylinder(unittest.TestCase):
    def test_cylinder(self):
        geo = GeographicApp()

        center = Coordinate(2, 4)
        in_bound = Position.from_coordinate(geo.pbd(center, 45, 700), 500)

        cylinder = VerticalCylinder(geo, Coordinate(2, 4), 500, 250, 750)

        # Check altitude
        self.assertIn(Position(2.0, 4.0, 500), cylinder)
        self.assertNotIn(Position(2.0, 4.0, 0), cylinder)
        self.assertNotIn(Position(2.0, 4.0, 1000), cylinder)

        # Check position
        self.assertIn(in_bound, cylinder.bounding_box)
        self.assertNotIn(in_bound, cylinder)
