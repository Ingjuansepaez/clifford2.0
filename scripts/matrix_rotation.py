#!/usr/bin/env python3

import numpy as np 
from math import sin, cos, radians

def matrix_rotation(rotation=[0,0,0] , in_radians=True , order = 'xyz'):

    roll = rotation[0]
    pitch = rotation[1]
    yaw = rotation[2]
    
    if not in_radians:
        roll = radians(roll)
        pitch = radians(pitch)
        yaw = radians(yaw)

    rx = np.matrix([[1,      0,          0   ],
                   [0, cos(roll), -sin(roll)],
                   [0, sin(roll), cos(roll) ] ])
    
    ry = np.matrix([[cos(pitch), 0, sin(pitch) ], 
                   [    0,     1,     0     ],
                   [-sin(pitch), 0, cos(pitch)] ])
    
    rz = np.matrix([[cos(yaw), -sin(yaw), 0 ],
                   [sin(yaw), cos(yaw),  0 ],
                   [    0,          0,       1 ] ])
    
    if order == "xyz":
        rotation_matrix = rx * ry * rz
    elif order == "xzy":
        rotation_matrix = rx * rz * ry
    elif order == "yzx":
        rotation_matrix = ry * rz * rx
    elif order == "zyx":
        rotation_matrix = rz * ry * rx
    elif order == "zxy":
        rotation_matrix = rz * rx * ry
    elif order == "xzy":
        rotation_matrix = rx * rz * ry
    
    return rotation_matrix
    
    
    
