# -*- coding: utf-8 -*-
"""
@author: Javier
"""
import modern_robotics as mr
import numpy as np
import configuration as config 
from numpy.linalg import pinv

def feedforwardPI_control (x,xd,xd_next,kp,ki,delta_t,cum_err):
    """
    Parameters
    ----------
    x : TYPE
        DESCRIPTION.
    xd : TYPE
        DESCRIPTION.
    xd_next : TYPE
        DESCRIPTION.
    kp : TYPE
        DESCRIPTION.
    ki : TYPE
        DESCRIPTION.
    delta_t : TYPE
        DESCRIPTION.
    cum_err : TYPE, optional
        DESCRIPTION. The default is 0.

    Returns
    -------
    vt : TYPE
        DESCRIPTION.
    xerr : TYPE
        DESCRIPTION.

    """
    x_inv = mr.TransInv(x)
    xd_inv = mr.TransInv(xd)
    ad_x_inv_xd = mr.Adjoint(np.dot(x_inv,xd))
    vd =mr.se3ToVec((1/delta_t)*mr.MatrixLog6(np.dot(xd_inv,xd_next)))
    xerr = mr.se3ToVec(mr.MatrixLog6(np.dot(x_inv,xd))) 
    cum_err = cum_err + np.dot(ki,xerr)*delta_t 
    vt = np.dot(ad_x_inv_xd,vd)+np.dot(kp,xerr)+cum_err
    return vt, xerr, cum_err

def commanded_speeds(je,vt):
    """
    Parameters
    ----------
    je : matrix (6x4)
        robot jacobian.
    vt : matrix (6x1)
        end -effector twist expressed in the end effector
    Returns
    -------
    arm_joint_speeds : matrix (4x1)
        The commanded arm joints velocities
        [j1,j2,j3,j4,j5]
    """
    #pseudoinverse of Je with 1e-3 tolerance to avoid singularity
    arm_joint_speeds = np.dot(pinv(je,1e-3),vt)
    
    return arm_joint_speeds

def test_joint_limits (theta_arm_list):
    '''
    Parameters
    ----------
    theta_arm_list : array (1x4)
        The current angle list (radians) of the arm in form [j1,j2,j3.j4].

    Returns
    -------
    limit_constraints : array (1x4)
        The list(True/False) of the arm angles constraints.
        If the actual angle is greater or less than the maximum 
        and minimum constraint respectively(configuration.py), then return True, 
        otherwise return False.
    '''
    j1_limits=config.arm_angles_limits[:,0]
    j2_limits=config.arm_angles_limits[:,1]
    j3_limits=config.arm_angles_limits[:,2]
    j4_limits=config.arm_angles_limits[:,3]
    limit_constraints = [False,False,False,False]
    
    if not(j1_limits[0]<theta_arm_list[0]<j1_limits[1]):
        limit_constraints[0] = True
    
    if not(j2_limits[0]<theta_arm_list[1]<j2_limits[1]):
        limit_constraints[1] = True
    
    if not(j3_limits[0]<theta_arm_list[2]<j3_limits[1]):
        limit_constraints[2] = True
        
    if not(j4_limits[0]<theta_arm_list[3]<j4_limits[1]):
        limit_constraints[3] = True
    
    return limit_constraints