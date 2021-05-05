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
        ## Add new grid
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
        return grid_