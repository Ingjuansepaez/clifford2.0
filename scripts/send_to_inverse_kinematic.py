#!/usr/bin/env python3
from inverse_kinematics_publisher import InverseKinematicsPublisher

kinematic = InverseKinematicsPublisher()

roll = 0
pitch = 0
yaw = 0
x = 0.8
y = 0.0
z = -0.5
rot_x = 0
rot_y = 0
rot_z = 0

def update(roll, pitch, yaw, x, y, z, rot_x, rot_y, rot_z):
    
    xyz = [x,y,z]
    rotation = [roll, pitch, yaw]
    center_offset = [rot_x, rot_y, rot_z]

    kinematic.compute_rotation_matrix_to_IK(xyz, rotation, 0, center_offset, is_radians=True)
