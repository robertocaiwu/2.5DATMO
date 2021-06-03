# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Create the tracker

"""
import numpy as np
from src.tools import utils as util

class Tracker:
    def __init__(self, settings_, max_age_=None, min_hit_streak_=None):