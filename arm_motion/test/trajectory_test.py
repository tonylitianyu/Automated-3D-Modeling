#!/usr/bin/env python
"""Unit testing for motion"""
import unittest
import trajectory
from trajectory import TurtleBot

class MotionTestCase(unittest.TestCase):
    def test_index_range(self):
        scan_side = 4
        turtle = TurtleBot(scan_side)
        self.assertEqual(scan_side,4)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(trajectory, "motion_test_case", MotionTestCase)