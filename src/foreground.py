# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Modeling the dynamic part of the environment by comparing cells on the current grid to neighboring cells on the background grid.

"""
import numpy as np
import scipy as sp
from src.tools import utils as util

class Foreground:
    def __init__(self, settings_):
        self.settings = settings_
        self.frg_grid = np.zeros(self.settings.grid_size)

    def create_motion_grid(self, bgr_grid, grid, rotation, translation):
        block_size = int(np.ceil(self.settings.neighbor_size/np.min([self.settings.cell_size_x, self.settings.cell_size_y])))
        frg_grid = np.zeros(self.settings.grid_size)
        for i in range(self.settings.grid_size_x):
            for j in range(self.settings.grid_size_y):
                # get indexes of neighbor cells
                Id = 0 if i - block_size < 0 else i - block_size
                Iu = self.settings.grid_size_x  if i + block_size > self.settings.grid_size_x else i + block_size
                Jd = 0 if j - block_size < 0 else j - block_size
                Ju = self.settings.grid_size_y  if j + block_size > self.settings.grid_size_y else j + block_size
                neighbour_blk = bgr_grid[Id:Iu, Jd:Ju]
                neighbour_blk = np.abs(np.subtract(neighbour_blk, grid.grid[i,j]))
                if np.min(neighbour_blk) > self.settings.neighbor_threshold * grid.grid[i,j]:
                    frg_grid[i,j] = grid.grid[i,j]
        # morphological operations (Dilation and fill holes) in x and y
        dx = np.ones([int(self.settings.dx/self.settings.cell_size_x),2])
        dy = np.ones([2, int(self.settings.dy/self.settings.cell_size_y)])
        sp.ndimage.morphology.binary_dilation(frg_grid, structure=dx).astype(frg_grid.dtype)
        sp.ndimage.morphology.binary_closing(frg_grid).astype(frg_grid.dtype)
        sp.ndimage.morphology.binary_dilation(frg_grid, structure=dy).astype(frg_grid.dtype)
        sp.ndimage.morphology.binary_closing(frg_grid).astype(frg_grid.dtype)
        # morphological operations (Dilation, fill holes, and erosion) in car direction: x
        dn = np.ones([int(self.settings.dn/self.settings.cell_size_x),1])
        sp.ndimage.morphology.binary_dilation(frg_grid, structure=dx).astype(frg_grid.dtype)
        sp.ndimage.morphology.binary_closing(frg_grid).astype(frg_grid.dtype)
        sp.ndimage.morphology.binary_erosion(frg_grid, structure=np.ones([3,3])).astype(frg_grid.dtype)

        self.frg_grid = frg_grid
        # convert matrix into points
        self.frg_points, self.frg_points_transformed = grid.convert_matrix_to_points(self.frg_grid, rotation, translation)






