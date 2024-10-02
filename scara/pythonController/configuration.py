# -*- coding: utf-8 -*-
"""
@author: Javier
"""
import modern_robotics as mr
import math as mt
import numpy as np

'''
SCARA robot (RRPR) configuration and limits

'''

# M = Tsb , zero configuration of {b} relative to {s}
M = np.array([[1, 0, 0, 1.73455],
              [0, 1, 0, 0      ],
              [0, 0, 1, 0.364  ],
              [0, 0, 0, 1      ]])

# screw axes B for the 4 joints expresed in the end effector frame {b}
B_list = np.array([[0,      0,      0, 0],
                   [0,      0,      0, 0],
                   [1,      1,      0, 1],
                   [0,      0,      0, 0],
                   [1.73455,0.80097,0, 0],
                   [0,      0,      -1,0]])

# [J1,J2,J3,J4] arm angles limits (rad)
arm_angles_limits = np.array([[-2.807,-2.4,0,    -3.142],
                              [ 2.807, 2.4,0.164, 3.142]])

# speed limits [arm joint in position mode(rad/s),arm joint in velocity mode(rad/s)] 
# this values were assumed!!!
speed_limits = np.array([[10,0.03],
                         [10,2.5]])


# cube initial position
omghat = np.array([0,0,1])
theta = -1 * (mt.pi/4)
p = np.array([1.735,0,0.1])
omgtheta = omghat*theta
R = mr.MatrixExp3(mr.VecToso3(omgtheta))
Tsc_init = mr.RpToTrans(R, p)

# cube final position
Tsc_final = np.array([[1,0,0,-0.715],
                      [0,1,0, 1.225],
                      [0,0,1, 0.1  ],
                      [0,0,0, 1    ]])

# save results as a csv file
def save_file(matrix, file_name="joints"):
    """
    Parameters
    ----------
    matrix : matrix nxm
        Save the matrix mxn as a .csv file with a ',' delimeter
    file_name : string
        default name "joints" or asign any other name
    """
    np.savetxt("results/"+file_name+".csv",matrix, delimiter=',', fmt='%.6f') 
    