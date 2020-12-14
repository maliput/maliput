#!/usr/bin/env python3
#
# Copyright 2020 Toyota Research Institute
#

"""Unit tests for the math python binding"""

import math as m
import unittest

from maliput.math import (
    RollPitchYaw,
    Quaternion,
    Vector3,
    Vector4,
)


class TestMaliput(unittest.TestCase):
    """
    Unit tests for the math python binding
    """
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

    def test_rollpitchyaw(self):
        '''
        Evaluates the constructor and accessors.
        '''
        kDut = RollPitchYaw(m.pi/2, 0., m.pi/2)
        self.assertTrue(kDut.roll_angle() == m.pi/2)
        self.assertTrue(kDut.pitch_angle() == 0.)
        self.assertTrue(kDut.yaw_angle() == m.pi/2)
        quat = kDut.ToQuaternion()
        kExpectedQuat = Quaternion(0.5, 0.5, 0.5, 0.5)
        self.assertAlmostEqual(quat.w(), kExpectedQuat.w())
        self.assertAlmostEqual(quat.x(), kExpectedQuat.x())
        self.assertAlmostEqual(quat.y(), kExpectedQuat.y())
        self.assertAlmostEqual(quat.z(), kExpectedQuat.z())

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
