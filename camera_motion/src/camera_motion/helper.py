
'''Helper class for depth image processing
'''

import math


class DepthHelper:

    def __init__(self, t_avg_range, s_avg_range):
        '''Init a depth helper class
            Args:
                t_avg_range (int) - take average for this time period
                s_avg_range (int) - take average for center s_avg_range * s_avg_range pixels
        '''
        self.t_avg_range = t_avg_range
        self.s_avg_range = s_avg_range
        

    def spaceSmoothing(self, image):
        '''take average depth in neighbor pixel
            Args:
                image (array) - incoming raw depth image
            Returns:
                avg_depth_space (float) - average depth value for center pixels
        '''        
        img_width = len(image)
        img_height = len(image[1])

        mid_width = int(img_width/2)
        mid_height = int(img_height/2)

        depth_sum = 0
        count_pixel = 0
        for i in range(mid_width-int(self.s_avg_range/2),mid_width+int(self.s_avg_range/2)):
            for j in range(mid_height-int(self.s_avg_range/2),mid_height+int(self.s_avg_range/2)):
                count_pixel += 1
                depth_sum += image[i][j]

        
        avg_depth_space = depth_sum/count_pixel
        return avg_depth_space

    def timeSmoothing(self, time_queue, next_depth):
        '''take average depth in time
            Args:
                time_queue (array) - depth value in the past that needs to taken into the average calculation
                next_depth (float) - average depth pixel at the center in space
            Returns:
                avg_depth_time (int) - average depth pixel in time
        '''

        if len(time_queue) >= self.t_avg_range:
            time_queue.pop(0)

        time_queue.append(next_depth)

        avg_depth_time = sum(time_queue)/len(time_queue)

        return int(avg_depth_time)