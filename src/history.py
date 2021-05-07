# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Modeling the static part of the environment by saving the last n grids and transformed to match the current pose.

"""

import numpy as np

class History:
    def __init__(self, settings_):
        self.settings = settings_
        self.stack = []

    def add_grid(self, grid_, rotation, translation):
        ## Add new grid into stack
        self.stack.append(grid_)
        if len(self.stack) > self.settings.max_history:
            self.stack.pop(0)
        ## transform old grids here
        if len(self.stack) > 1
            for i in range(len(self.stack)-1)
                previous_grid = self.stack[i]

                transformed_grid = transform_grid(previous_grid, rotation, translation)

                self.stack[i] = transformed_grid

    def transform_grid(self, grid_, rotation, translation):

        old_points = np.zeros(grid_.grid_points.shape)
        # move the map [frm] (mat.ptn) to the origin by transformation of [frm + 1] (pts.trn, pts.rtn)
        old_points = np.subtract(grid_.grid_points_transformed.T, translation) #trajectory at frm + 1: translation
        old_points = rotation.T.dot(old_points.T)                     # rotation
        imz = history[0].grid.reshape(history[0].grid_size[0]*history[0].grid_size[1],)
        old_points[2,:] = imz[:]                                               # put map values on corresponding locations:[x y values]

        #  move the map [x, y] to the matrix coordinate [i, j]
        old_points[0:2,:] = np.divide(old_points[0:2,:], \ 
                                      np.tile([self.settings.cell_size_x, \
                                               self.settings.cell_size_y], \
                                               [self.settings.grid_size,1]).T)
        old_points[0:2,:] =  np.floor(np.add(old_points[0:2,:], \
                                      np.tile([-grid.settings.grid_xb/grid.settings.cell_size_x, \
                                               -grid.settings.grid_yr/grid.settings.cell_size_y], \
                                                [self.settings.grid_size,1]).T))


        valid_idx = np.logical_and(np.logical_and(old_points[0,:] >= 0 ,old_points[0,:] <= ((self.settings.grid_xf - self.settings.grid_xb) / self.settings.cell_size_x - 1)), \
                                np.logical_and(old_points[1,:] >= 0 ,old_points[1,:] <= ((self.settings.grid_yl - self.settings.grid_yr) / self.settings.cell_size_y - 1)) )
        #  keep valid values [valid indexes]
        valid_points = old_points[:,valid_idx]
        
        # convert points (vectors) to matrix with valid index
        transformed_grid =  np.zeros(grid.grid_size)
        for i in range(valid_points.shape[1]):
            
            x = int(valid_points[0,i])
            y = int(valid_points[1,i])
            z = valid_points[2,i]
            transformed_grid[x, y] = z
        return grid_