# -*- coding: utf-8 -*-
"""
Created on 2021.04.30

Author: Roberto Cai

Several helper methods.

"""
import numpy as np

def rotx(t):
    """Rotation about the x-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1,  0,  0],
                     [0,  c, -s],
                     [0,  s,  c]])


def roty(t):
    """Rotation about the y-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  0,  s],
                     [0,  1,  0],
                     [-s, 0,  c]])


def rotz(t):
    """Rotation about the z-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s,  0],
                     [s,  c,  0],
                     [0,  0,  1]])

def pose_from_oxts_packet(packet):
    """Helper method to compute a SE(3) pose matrix from an OXTS packet.
    """
    lat, lon, alt, roll, pitch, yaw = packet[0:6]
    print(lat, lon, alt, roll, pitch, yaw)
    scale = np.cos(lat * np.pi / 180.)
    
    er = 6378137.  # earth radius (approx.) in meters

    # Use a Mercator projection to get the translation vector
    tx = scale * lon * np.pi * er / 180.
    ty = scale * er * \
        np.log(np.tan((90. + lat) * np.pi / 360.))
    tz = alt
    t = np.array([tx, ty, tz])

    # Use the Euler angles to get the rotation matrix
    Rx = rotx(roll)
    Ry = roty(pitch)
    Rz = rotz(yaw)
    R = Rz.dot(Ry.dot(Rx))

    # Combine the translation and rotation into a homogeneous transform
    return R, t


def get_distance_to_point(point1, point2):
    return np.sqrt(get_distance_to_point_squared(point1, point2))
    
def get_distance_to_point_squared(point1, point2):
    return (point2[0] - point1[0])**2 + (point2[1] - point1[1])**2 + (point2[2] - point1[2])**2

def increase_bbox(bbox, width, height):
    """
    Increases the width and height of the bounding box
    """
    bbox[0] -= width
    bbox[1] += width
    bbox[2] -= height
    bbox[3] += height
    return bbox

def convert_bbox_to_observation(bbox):
    """
    Takes a bounding box in the form [x1,x2,y1,y2] and returns z in the form
    [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
    the aspect ratio
    """
    w = bbox[1] - bbox[0]
    h = bbox[3] - bbox[2]
    x = bbox[0] + w/2.
    y = bbox[2] + h/2.
    s = w * h    #scale is just area
    r = w / float(h)
    return np.array([x, y, s, r]).reshape((4, 1))

def convert_detection_to_bbox(detection):
    """
    Takes a bounding box in the form  [x1,x2,y1,y2] and returns z in the form
    [x,y,s,r] where x,y is the centre of the box and w is the width and h is
    the height
    """
    w = int(detection[1] - detection[0])
    h = int(detection[3] - detection[2])
    x = int(detection[0])
    y = int(detection[2])
    return [x, y, w, h]


def linear_assignment(cost_matrix):
    try:
        import lap
        _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
        return np.array([[y[i],i] for i in x if i >= 0]) #
    except ImportError:
        from scipy.optimize import linear_sum_assignment
        x, y = linear_sum_assignment(cost_matrix)
        return np.array(list(zip(x, y)))