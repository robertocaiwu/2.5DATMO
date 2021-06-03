# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Detection and tracking of objects.

"""
import numpy as np
from src.tools import utils as util

class DATMO:
    def __init__(self, settings_, max_age_=None, min_hit_streak_=None):
        self.settings = settings_
        self.max_age = max_age_
        self.min_hit_streak = min_hit_streak_
        self.frame_count
        self.tracker = 


    def update(self, detections):
        self.frame_count += 1

        matched_det = []
        unamtched_det = []
        tracked_det = []