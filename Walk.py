from numpy import *
from matplotlib.pyplot import *


class Walk:
    def __init__(self,stride_length=0.03,stride_height=0.01):
        self.stride_height = stride_height
        self.stride_length = stride_length
        self.p = array([0, 12.5,25,37.5,50,62.5,75,87.5,100])
        self.FR_x = array([0,-self.stride_length/2,-self.stride_length/2,0,self.stride_length/2,self.stride_length,self.stride_length])