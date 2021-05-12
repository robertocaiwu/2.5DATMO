# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Modeling the dynamic part of the environment by comparing cells on the current grid to neighboring cells on the background grid.

"""

import numpy as np

class Foreground:
    def __init__(self, settings_):
        self.settings = settings_
        self.frg_grid = np.zeros(self.settings.grid_size)

    def create_motion_grid(bgr_grid, grid)
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
                if np.min(neighbour_blk) > self.neighbor_threshold * grid.grid[i,j]:
                    frg_grid[i,j] = grid.grid[i,j]
        self.frg_grid = frg_grid

