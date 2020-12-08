#!/usr/bin/env python
"""Unit testing for smoothing functions"""
import unittest
import camera_motion
from camera_motion.helper import DepthHelper


class OutputTestCase(unittest.TestCase):
    def test_smoothing_space(self):
        '''test average depth value in space
        '''


        helper = DepthHelper(20, 2)
        image = [[0,0,0,0],[0,1,1,0],[0,1,1,0],[0,0,0,0]]
        avg_depth_space = helper.spaceSmoothing(image)
        self.assertEquals(avg_depth_space,1.0)


    def test_smoothing_time(self):
        '''test average depth value in time
        '''
        helper = DepthHelper(20, 2)
        time_queue = [689,690,691,692,693]
        next_depth = 694
        avg_depth_time = helper.timeSmoothing(time_queue, next_depth)

        self.assertEquals(avg_depth_time, 691)



if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(camera_motion, "output_test_case", OutputTestCase)