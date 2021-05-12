# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Modeling the static part of the environment by saving the last n grids and transformed to match the current pose.

"""

import numpy as np
from src import grids as gr

class History:
    def __init__(self, settings_):
        self.settings = settings_
        self.stack = []
        # self.background = gr.Grid(self.settings)
        self.bgr_grid = np.zeros(self.settings.grid_size)
        self.bgr_grid_conf  = np.zeros(self.settings.grid_size)

    def model_history(self, grid_, rotation, translation):
        self.add_grid(grid_, rotation, translation)

        bgr_grid = np.zeros(self.settings.grid_size)  
        bgr_grid_conf = np.zeros(self.settings.grid_size)  

        for i in range(self.settings.grid_size_x):
            for j in range(self.settings.grid_size_y):
                if np.sum([grid.grid_valid[i,j] != 0 for grid in self.stack]):                    # if that cell has any valid data on it 
                    idx_valid = [grid.grid_valid[i,j] != 0 for grid in self.stack]            
                    if np.sum(idx_valid) > self.settings.cells_to_integrate:
                        cells_to_integrate = np.where(idx_valid)
                        idx_valid[cells_to_integrate[self.settings.cells_to_integrate:]] = False
                    cells_to_avg = np.where(idx_valid)
                    bgr_grid[i,j] = np.mean([self.stack[c].grid[i,j] for c in cells_to_avg[0].tolist()])
                    bgr_grid_conf[i,j] = np.sum(idx_valid)
        self.bgr_grid = np.multiply((bgr_grid_conf >= self.settings.minimum_observations),bgr_grid)
        self.bgr_grid_conf = bgr_grid_conf
        # convert matrix into points
        self.bgr_points, self.brg_points_transformed = grid_.convert_matrix_to_points(self.bgr_grid, rotation, translation)

    def add_grid(self, grid_, rotation, translation):
        ## Add new grid into stack
        self.stack.append(grid_)
        if len(self.stack) > self.settings.max_history:
            self.stack.pop(0)
        ## transform old grids here
        if len(self.stack) > 1:
            for i in range(len(self.stack)-1):

                previous_grid = self.stack[i]
                grid = gr.Grid(self.settings)

                grid.grid, grid.grid_points, grid.grid_points_transformed = self.transform_grid(previous_grid, rotation, translation)
                previous_grid.grid = previous_grid.grid_valid
                grid.grid_valid, _, _ = self.transform_grid(previous_grid, rotation, translation)

                self.stack[i].grid = grid.grid
                self.stack[i].grid_valid = grid.grid_valid
                self.stack[i].grid_points = grid.grid_points
                self.stack[i].grid_points_transformed = grid.grid_points_transformed

    def transform_grid(self, grid_, rotation, translation):
        # Empty grid and point arrays
        grid = np.zeros(self.settings.grid_size)  
        old_points = np.zeros(grid_.grid_points.shape)

        # move the map [frm] (mat.ptn) to the origin by transformation of [frm + 1] (pts.trn, pts.rtn)
        old_points = np.subtract(grid_.grid_points_transformed.T, translation) #trajectory at frm + 1: translation
        old_points = rotation.T.dot(old_points.T)                     # rotation
        imz = grid_.grid.reshape(self.settings.num_cells,)
        old_points[2,:] = imz[:]                                               # put map values on corresponding locations:[x y values]

        #  move the map [x, y] to the matrix coordinate [i, j]
        old_points[0:2,:] = np.divide(old_points[0:2,:], 
                                      np.tile([self.settings.cell_size_x, self.settings.cell_size_y], 
                                              [self.settings.num_cells,1]).T)
        old_points[0:2,:] =  np.floor(np.add(old_points[0:2,:], 
                                      np.tile([-self.settings.grid_xb/self.settings.cell_size_x, 
                                               -self.settings.grid_yr/self.settings.cell_size_y], 
                                                [self.settings.num_cells,1]).T))

        #  keep valid values [valid indexes]
        valid_idx = np.logical_and(np.logical_and(old_points[0,:] >= 0 , \
                                                  old_points[0,:] <= ((self.settings.grid_xf - self.settings.grid_xb) / self.settings.cell_size_x - 1)), \
                                   np.logical_and(old_points[1,:] >= 0 , \
                                                  old_points[1,:] <= ((self.settings.grid_yl - self.settings.grid_yr) / self.settings.cell_size_y - 1)) )

        valid_points = old_points[:,valid_idx]
        
        # convert points (vectors) to matrix with valid index
        transformed_grid =  np.zeros(self.settings.grid_size)
        for i in range(valid_points.shape[1]):
            
            x = int(valid_points[0,i])
            y = int(valid_points[1,i])
            z = valid_points[2,i]
            transformed_grid[x, y] = z

        # convert matrix to points
        grid_points, grid_points_transformed = grid_.convert_matrix_to_points(transformed_grid, rotation, translation)
        grid[:,:] = transformed_grid[:,:]
        return grid, grid_points, grid_points_transformed