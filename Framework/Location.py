import random
import numpy as np
import math


class Location:

    def __init__(self, radius, indoor=False, gateway=False):
        # redid the implementation
        # uniform distribution
        #self.x = random.uniform(min, max)
        #self.y = random.uniform(min, max)
        if gateway:
            self.x = 0
            self.y = 0
        else:
            a = random.random()
            b = random.random()
            phi = 2 * a * math.pi  
            r = math.sqrt(b)
            self.x = radius * r * math.cos(phi) 
            self.y = radius * r * math.sin(phi) 
        self.indoor = indoor

    @staticmethod
    def distance(loc_1, loc_2):
        delta_x = loc_1.x - loc_2.x
        delta_y = loc_1.y - loc_2.y
        return np.sqrt(np.power(delta_x, 2) + np.power(delta_y, 2))
