# -*- coding: utf-8 -*-
"""
@author: Javier
"""
import configuration as config
import kinematics as kinematics
#import trajectory as trajectory
import controller as controller
import numpy as np
import modern_robotics as mr
import math as math

# controller data
kp_const = 1.2
ki_const = 0.2
kp = np.eye(6)*kp_const
ki = np.eye(6)*ki_const
delta_t = 0.01
# trajectory
file_name = "screw_trajectory"
full_trajectory = np.loadtxt("../results/"+file_name+".csv",delimiter=",")
# arm configuration
init_angles = [0,0,0,0]
# save data
csv_arm_motion = np.empty((len(full_trajectory)-1,5),float)
csv_error_motion = np.empty((len(full_trajectory)-1,6),float)

# Functions--------------------------------------------------------------------
def Tse_current(theta_arm_list):
    """
    Parameters
    ----------
    theta_arm_list : TYPE
        DESCRIPTION.
    Returns
    -------
    Tse : TYPE
        DESCRIPTION.
    """
    Tse = mr.FKinBody(config.M, config.B_list, init_angles)
    return Tse

def T_matrix(R_list,p_list):
    """
    Parameters
    ----------
    R_list : TYPE
        DESCRIPTION.
    p_list : TYPE
        DESCRIPTION.

    Returns
    -------
    T : matrix (4x4)
        Transformation matrix
    """
    R = R_list.reshape(3,3)
    p = p_list.reshape(3,1)
    T_last_row = np.array([[0,0,0,1]])
    Rp = np.concatenate([R,p],axis=1)
    T = np.concatenate([Rp,T_last_row],axis=0)
    return T
#------------------------------------------------------------------------------

# Main program-----------------------------------------------------------------
cum_error = 0
for i in range(len(full_trajectory)-1):
    x = Tse_current(init_angles)
    xd = T_matrix(full_trajectory[i,0:9], full_trajectory[i,9:12])
    xd_next = T_matrix(full_trajectory[i+1,0:9], full_trajectory[i+1,9:12])
    vt, xerr,cum_error = controller.feedforwardPI_control(x,xd,xd_next,kp,ki,delta_t,cum_error)
    Je = kinematics.robot_jacobian(init_angles)
    arm_joint_limits = controller.test_joint_limits(init_angles)  
    

    if(any(arm_joint_limits)):
        for j in range(len(arm_joint_limits)):
            if(arm_joint_limits[j]==True):
                Je[:,j] = 0
    
    arm_joint_speed = controller.commanded_speeds(Je,vt)  
    new_state = kinematics.next_state(init_angles,
                                      arm_joint_speed,
                                      delta_t,
                                      list(config.speed_limits[:,0]))
    init_angles = new_state
    csv_arm_motion[i,:-1] = new_state
    csv_arm_motion[i,-1] = full_trajectory[i,-1]
    csv_error_motion[i] = xerr


config.save_file(csv_arm_motion,"arm_motion")
config.save_file(csv_error_motion,"arm_controller_error")


#------------------------------------------------------------------------------

