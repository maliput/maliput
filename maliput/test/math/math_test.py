import unittest

from maliput.math import (
  Quaternion,
  Vector3,
  Vector4,
)

class TestMaliput(unittest.TestCase):

    def test_vector3(self):
        '''
        Evaluates the constructor and accessors.
        '''
        kDut = Vector3(25., 158., 33.)
        self.assertTrue(kDut.size() == 3.)
        self.assertTrue(kDut.x() == 25.)
        self.assertTrue(kDut.y() == 158.)
        self.assertTrue(kDut.z() == 33.)
        self.assertTrue(kDut[0] == 25.)
        self.assertTrue(kDut[1] == 158.)
        self.assertTrue(kDut[2] == 33.)
        self.assertTrue(kDut == Vector3(25., 158., 33.))
        self.assertTrue(kDut != Vector3(33., 158., 25.))

    def test_vector4(self):
        '''
        Evaluates the constructor and accessors.
        '''
        kDut = Vector4(25., 158., 33., 0.02)
        self.assertTrue(kDut.size() == 4.)
        self.assertTrue(kDut.x() == 25.)
        self.assertTrue(kDut.y() == 158.)
        self.assertTrue(kDut.z() == 33.)
        self.assertTrue(kDut.w() == 0.02)
        self.assertTrue(kDut[0] == 25.)
        self.assertTrue(kDut[1] == 158.)
        self.assertTrue(kDut[2] == 33.)
        self.assertTrue(kDut[3] == 0.02)
        self.assertTrue(kDut == Vector4(25., 158., 33., 0.02))
        self.assertTrue(kDut != Vector4(0.02, 33., 158., 25.))

    def test_quaternion(self):
        '''
        Evaluates the constructor and accessors.
        '''
        kDut = Quaternion(0.884, 0.306, 0.177, 0.306)
        self.assertTrue(kDut.w() == 0.884)
        self.assertTrue(kDut.x() == 0.306)
        self.assertTrue(kDut.y() == 0.177)
        self.assertTrue(kDut.z() == 0.306)
        self.assertTrue(kDut.coeffs() == Vector4(0.884, 0.306, 0.177, 0.306))
