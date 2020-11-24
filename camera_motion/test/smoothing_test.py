#!/usr/bin/env python
"""Unit testing for smoothing functions"""
import unittest
import camera_motion
from camera_motion.helper import DepthHelper


class OutputTestCase(unittest.TestCase):
    def test_smoothing_space(self):
        self.assertEquals(0.0,0.0)



if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(camera_motion, "output_test_case", OutputTestCase)