#!/usr/bin/env python3
#Librerias de ROS2
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from matrix_rotation import matrix_rotation
from numpy.linalg import inv
from numpy import array, asarray

#Librerias de Python
import math
import time
import numpy as np  

class InverseKinematicsPublisher(Node):
    def __init__(self, xyz, rotation, center_offset):
        super().__init__('inverse_kinematic_publisher')
        
        self.xyz = xyz
        self.rotation_xyz = rotation
        self.center_offset_xyz = center_offset
            
        self.names_of_joints = [
            "front_left_wrist_joint",
            "front_left_leg_joint",
            "front_left_foot_joint",
            "back_left_wrist_joint",
            "back_left_leg_joint",
            "back_left_foot_joint",
            "front_right_wrist_joint",
            "front_right_leg_joint",
            "front_right_foot_joint",
            "back_right_wrist_joint",
            "back_right_leg_joint",
            "back_right_foot_joint"
        ]

        self.l1 = 0.05
        self.l2 = 0.12
        self.l3 = 0.14
        
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        #self.lenght = 0.10
        self.lenght = 0.16
        self.wight = 0.24

        self.leg_origin = np.matrix([[self.lenght/2, self.wight/2, 0],
                                    [-self.lenght/2, self.wight/2, 0],
                                    [self.lenght/2, -self.wight/2, 0],
                                    [-self.lenght/2, -self.wight/2, 0],
                                    [self.lenght/2, self.wight/2, 0]])
        
        self.leg_left_front = 0
        self.leg_left_back = 1
        self.leg_right_front = 2
        self.leg_right_back = 3

        self.leg_position = [self.leg_left_front, self.leg_left_back]

        self.angle_publisher = self.create_publisher(
            JointState, 
            '/joint_states',
            10
        )

        self.publish_values_in_joints()
        
    def publish_values_in_joints(self):
        
        # Compute desired x,y,z for each leg 
        xyz = self.compute_rotation_matrix_to_IK(self.xyz, self.rotation_xyz, 0, self.center_offset_xyz, True)
        print(f"x: {xyz[0]}")
        print(f"y: {xyz[1]}")
        print(f"z: {xyz[2]}")

        # getting the qs for each leg
        self.compute_inverse_kinematics(xyz)

        #assigning each leg's qs to the URDF 
        front_left_q1 = self.q1
        front_left_q2 = self.q2
        front_left_q3 = self.q3
        back_left_q1 = self.q1
        back_left_q2 = self.q2
        back_left_q3 = self.q3
        front_right_q1 = self.q1
        front_right_q2 = self.q2
        front_right_q3 = self.q3
        back_right_q1 = self.q1
        back_right_q2 = self.q2
        back_right_q3 = self.q3

        self.values_in_joints = [
            front_left_q1,
            front_left_q2, 
            front_left_q3,
            back_left_q1,
            back_left_q2,
            back_left_q3,
            front_right_q1,
            front_right_q2, 
            front_right_q3,
            back_right_q1,
            back_right_q2,
            back_right_q3,
        ]

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.names_of_joints
        joint_state.position = self.values_in_joints
        
        while True:
            self.angle_publisher.publish(joint_state)

    def compute_rotation_matrix_to_IK(self, xyz, rot=[0,0,0], legID=0, center_offset=[0,0,0], is_radians=True):

        XYZ = asarray((inv(matrix_rotation(rot, is_radians))* \
                ((array(xyz) + self.leg_origin[legID, :] - array(center_offset)).transpose())).transpose())
        
        xyz = asarray(XYZ - self.leg_origin[legID, :] + array(center_offset)).flatten()

        return xyz

    def compute_inverse_kinematics(self, xyz):

        x = xyz[0]
        y = xyz[1]
        z = xyz[2]

        #------------------------------------------------------
        #PRIMERA PARTE DE LA CINEMATICA
        #------------------------------------------------------

        #------------------------------------------------------
        #SEGUNDA PARTE DE LA CINEMATICA
        #------------------------------------------------------

        a = math.sqrt((x)**2 + (z - self.l1)**2)

        beta = math.atan2((z - self.l1), math.sqrt((x**2)+(z**2)))
        cos_q3 = ((self.l3**2)+(self.l2**2)-(a**2))/(2*self.l2*self.l3)
        cos_alfa = ((self.l2**2)+(a**2)-(self.l3**2))/(2*self.l2*a)

        sen_q3 = math.sqrt(1-(cos_q3)**2)
        sen_alfa = math.sqrt(1-(cos_alfa)**2)

        self.q3 = math.atan2(sen_q3, cos_q3)
        alfa = math.atan2(sen_alfa, cos_alfa)

        self.q2 = -alfa - beta 

def main(args=None):
    rclpy.init(args=args)

    roll = 0
    pitch = 0
    yaw = 0
    x = 0.11
    y = 0.0
    z = 0.0
    rot_x = 0
    rot_y = 0
    rot_z = 0

    xyz = [x,y,z]
    rotation = [roll, pitch, yaw]
    center_offset = [rot_x, rot_y, rot_z]

    inverse_kinematics_publisher_node = InverseKinematicsPublisher(xyz, rotation, center_offset)
    
    
    try:
        rclpy.spin(inverse_kinematics_publisher_node)
    except KeyboardInterrupt:
        inverse_kinematics_publisher_node.destroy_node()
        rclpy.try_shutdown()

    
if __name__ == '__main__':
    main()


