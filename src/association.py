# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Modeling the dynamic part of the environment by comparing cells on the current grid to neighboring cells on the background grid.

"""
import numpy as np
from src.tools import utils as util

class Association:
    def __init__(self, settings_):
        self.settings = settings_
        self.clusters = []
        self.centroids = []
        self.boxes = []

    def cluster_points(self, points, cluster_distance_threshold=0.25, min_cluster_size=2):
        clusters = []
        remaining_points = []
        point = np.zeros(3)
        for i in range(points.shape[1]):
            if points[2, i] > 0:
                point = np.asarray([points[0, i], points[1, i], points[2, i]])
                remaining_points.append(point)
        while len(remaining_points) > 0: 
            cluster = []
            fringe = []
            fringe.append(remaining_points.pop(0))
            while len(fringe) > 0:
                point = fringe.pop(0)
                cluster.append(point)
                i = 0
                for p in remaining_points:
                    if util.get_distance_to_point(point, p) < cluster_distance_threshold:
                        fringe.append(p)
                        remaining_points.pop(i)
                    else:
                        i = i + 1
            if len(cluster) > min_cluster_size:
                clusters.append(cluster)
        self.clusters = clusters

    def get_cluster_centroids(self, clusters):
        centroids = []
        boxes = []
        for cluster in clusters:
            xmin = np.min([p[0] for p in cluster])
            xmax = np.max([p[0] for p in cluster])
            ymin = np.min([p[1] for p in cluster])
            ymax = np.max([p[1] for p in cluster])
            zmin = 0.
            zmax = np.max([p[2] for p in cluster])
            center = np.asarray([(xmax + xmin)/2, (ymax + ymin)/2, (zmax + zmin)/2])
            centroids.append(center)
            box = np.asarray([xmin, xmax, ymin, ymax, zmin, zmax])
            boxes.append(box)
        self.centroids = centroids
        self.boxes = boxes

    def get_point_as_index(self, points):
        point_indexes = []
        for p in points:
            idx = np.divide(p[0:2], np.asarray([self.settings.cell_size_x, self.settings.cell_size_y]))
            idx = np.floor(np.add(idx, np.asarray([-self.settings.grid_xb/self.settings.cell_size_x, 
                                                   -self.settings.grid_yr/self.settings.cell_size_y])))
            point_indexes.append(idx)
        return point_indexes

    def get_box_as_index(self, boxes):
        box_indexes = []
        for b in boxes:
            idx = np.divide(b[0:4], np.asarray([self.settings.cell_size_x, self.settings.cell_size_x, 
                                                self.settings.cell_size_y, self.settings.cell_size_y]))
            idx = np.floor(np.add(idx, np.asarray([-self.settings.grid_xb/self.settings.cell_size_x, 
                                                   -self.settings.grid_xb/self.settings.cell_size_x, 
                                                   -self.settings.grid_yr/self.settings.cell_size_y, 
                                                   -self.settings.grid_yr/self.settings.cell_size_y])))
            box_indexes.append(idx)
        return box_indexes
            
    def associate_detections_to_trackers(self, detections, trackers, threshold = 0.3):
        """
        Assigns detections to tracked object 
        Returns 3 lists of matches, unmatched_detections and unmatched_trackers
        """
        if(len(trackers)==0):
            return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)

        cost_matrix = iou_batch(detections, trackers)

        if min(iou_matrix.shape) > 0:
            a = (iou_matrix > threshold).astype(np.int32)
            if a.sum(1).max() == 1 and a.sum(0).max() == 1:
                matched_indices = np.stack(np.where(a), axis=1)
            else:
                matched_indices = linear_assignment(-iou_matrix)
        else:
            matched_indices = np.empty(shape=(0,2))

        unmatched_detections = []
        for d, det in enumerate(detections):
            if(d not in matched_indices[:,0]):
                unmatched_detections.append(d)
        unmatched_trackers = []
        for t, trk in enumerate(trackers):
            if(t not in matched_indices[:,1]):
                unmatched_trackers.append(t)

        #filter out matched with low IOU
        matches = []
        for m in matched_indices:
            if(iou_matrix[m[0], m[1]]<threshold):
                unmatched_detections.append(m[0])
                unmatched_trackers.append(m[1])
            else:
                matches.append(m.reshape(1,2))
        if(len(matches)==0):
            matches = np.empty((0,2),dtype=int)
        else:
            matches = np.concatenate(matches,axis=0)

        return matches, np.array(unmatched_detections), np.array(unmatched_trackers)

    