#!/usr/bin/env python
import numpy as np
import math
import random
import rospy

class RandomFlyCommander:
    def __init__(self, x_size, y_size, z_size, base_height=0.5):
        self.x_size = x_size
        self.y_size = y_size
        self.z_size = z_size
        self.base_height = base_height

        self.home_x = 0
        self.home_y = 0
        self.home_z = 0
    
    def control_update(self):
        pass
    
    def generate_next_point(self):
        pass
    


if __name__ == "__main__":
    random_fly = RandomFlyCommander(1.0, 1.0, 0.5)