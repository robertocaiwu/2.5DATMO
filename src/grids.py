# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Transforms a point cloud into a 2.5D grid as described in:
Asvadi, A., Peixoto, P., & Nunes, U. (2015). Detection and Tracking of Moving Objects Using 2.5D Motion Grids. 
IEEE Conference on Intelligent Transportation Systems, Proceedings, ITSC, 2015-Octob, 788–793. 
https://doi.org/10.1109/ITSC.2015.133

"""

import numpy as np

class Grid:
    def __init__(self, settings_, rotation, translation):
        self.settings = settings_
        self.grid_size = [self.settings.grid_size_x, self.settings.grid_size_y] 
        self.grid = np.zeros(self.grid_size)  
        self.grid_max = np.zeros(self.grid_size) # max 
        self.grid_avg = np.zeros(self.grid_size) # average
        self.grid_var = np.zeros(self.grid_size)  # variance
        self.grid_valid = np.zeros(self.grid_size) # valid data binary  
        self.grid_histogram = np.zeros(self.grid_size) # Histogram: number of tall points for object cells  
        self.object_grid = np.zeros(self.grid_size)  # valid objects
        self.R = rotation
        self.t = translation

    def build_height_grid(self, point_cloud):
        # quantize the index of every point 
        idx_quantizized = np.floor([point_cloud[0,:]/self.settings.cell_size_x, point_cloud[1,:]/self.settings.cell_size_y])
        # transform point's index to (-x, x) and (-y, y) 
        idx_quantizized_trans = [idx_quantizized[0,:] - self.settings.grid_xb/self.settings.cell_size_x,
                                idx_quantizized[1,:] - self.settings.grid_yr/self.settings.cell_size_y]
        cell_points = []
        # create grids %% results: maximum, mean, variance, valid data binary  
        for i in range(self.settings.grid_size_x):
            for j in range(self.settings.grid_size_y):
                cell_index = (i * self.settings.grid_size_y) + j
                idx = (idx_quantizized_trans[:][0] == i) & (idx_quantizized_trans[:][1] == j) 
                if np.sum(idx) != 0.:
                    cell_points.append(np.asarray(point_cloud[2,idx]))
                    self.grid_max[-i,-j] = np.max(cell_points[cell_index])                      # max 
                    self.grid_avg[-i,-j] = np.mean(cell_points[cell_index])                 # average
                    self.grid_var[-i,-j] = np.var(cell_points[cell_index])                  # variance
                    self.grid_valid[-i,-j] = 1. if np.any(cell_points[cell_index]) else 0.  # valid data binary  
                else:
                    cell_points.append([])

        # remove road: indexes for 'low variance', 'low height' and 'object'(non-road) cells 
        low_var_grid = (self.grid_var < self.settings.road_variance)
        low_height_grid = (self.grid_max < self.settings.road_max)
        self.object_grid = ~(np.multiply(low_var_grid,low_height_grid)) # valid objects (road blocks should have variance and height lower than a threshold)

        # refinement of object indexs: strengthen non-statistical feature of max by filtering out weak objects
        for i in range(self.settings.grid_size_x):
            for j in range(self.settings.grid_size_y):
                cell_index = (i * self.settings.grid_size_y) + j
                if self.object_grid[-i,-j] != 0:
                    self.grid_histogram[-i,-j] = np.sum(np.asarray(cell_points[cell_index]) >= self.settings.road_max)
                    if self.grid_histogram[-i,-j] != 0 and self.grid_histogram[-i,-j] < self.settings.grid_tr:
                        self.object_grid[-i,-j] = False

        # convert matrix into points
        self.grid = np.multiply(self.object_grid, self.grid_avg)
        imx, imy = np.meshgrid(range(self.grid_size[0]), range(self.grid_size[1]), indexing='xy')
        imx = imx.reshape(self.grid_size[0]*self.grid_size[1],)
        imy = imy.reshape(self.grid_size[0]*self.grid_size[1],)
        imz = self.grid.reshape(self.grid_size[0]*self.grid_size[1],)
        self.grid_points = np.asarray([imx[:] * self.settings.cell_size_x + self.settings.grid_xb, \
                                       imy[:] * self.settings.cell_size_y + self.settings.grid_yr, \
                                       imz[:]])
        rotated_points = self.R.dot(self.grid_points[:,:])
        self.grid_points_transformed = np.asarray([rotated_points[0,:] + self.t[0], \
                                                   rotated_points[1,:] + self.t[1], \
                                                   rotated_points[2,:] + self.t[2]])

