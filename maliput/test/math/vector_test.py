import unittest
import numpy as np

from maliput.math import Vector3

class TestMaliput(unittest.TestCase):
    def test_math(self):
        kDut = Vector3(25., 158., 33.)
        self.assertTrue(kDut.x(), 25.)
        self.assertTrue(kDut.y(), 158.)
        self.assertTrue(kDut.z(), 33.)
        self.assertTrue(kDut[0], 25.)
        self.assertTrue(kDut[1], 158.)
        self.assertTrue(kDut[2], 33.)
