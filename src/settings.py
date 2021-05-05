# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Settings for building 2.5D motion grid

"""

import numpy as np

class Settings:
    def __init__(self):
        
        # local grid options
        self.grid_xf   = 40;                                                        # x direction and front (x: -35 ~ +55)
        self.grid_xb   = -20;                                                       # x direction and behind
        self.grid_yl   = +12;                                                       # left and right (y: -15 ~ +15)
        self.grid_yr   = -12; 
        self.grid_zu   = 2.5;                                                       # z direction and up (z: 0 ~ +2.5)
        self.grid_zd   = -1;                                                        # z direction and down
        self.grid_nxf  = +30;                                                       # new x direction and front (x: -35 ~ +55)
        self.grid_nxb  = -10;                                                       # new x direction and behind
        self.grid_nyl  = +10;                                                       # new left and right (y: -15 ~ +15)
        self.grid_nyr  = -10; 
        self.grid_bs   = 3;                                                         # blind spot radius
        self.grid_tr   = 1;                                                         # minimum number of points to make a valid voxel
        
        # cell size
        self.cell_size_x    = 0.2;                                                  # 0.2m
        self.cell_size_y    = 0.2;                                                  # 0.2m
        self.grid_size_x   = int((self.grid_xf - self.grid_xb) / self.cell_size_x); 
        self.grid_size_y   = int((self.grid_yl - self.grid_yr) / self.cell_size_y); 

        # For ground cell removal
        self.height_bias = 1.73
        self.road_variance = 0.02
        self.road_max = 0.8

        # History
        self.max_history = 90