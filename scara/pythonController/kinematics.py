# -*- coding: utf-8 -*-
"""
@author: Javier
"""
import modern_robotics as mr
import configuration as config

def euler_step(angles,speed,limits,delta_t):
    """
    Parameters
    ----------
    angles : float
        The actual angle value.
    speed : float
        The actual speed value(rad/s).
    limits : matrix 2x1
        [0] min speed value , [1] max spped value.
    delta_t : float
        A time-step value.
        
    Returns
    -------
    new_angle : float 
        New angle after an euler step.
    """
    if speed < limits[0]:
        speed = limits[0]
    
    elif speed > limits[1]:
        speed = limits[1]
        
    new_angle = angles + speed*delta_t
    return new_angle 

def next_state(arm_angles,arm_joint_speed,delta_t,arm_speed_limits):
    """
    Parameters
    ----------
    arm_angles : array (1x4)
        Actual angles array [j1,j2,j3,j4]
        j1...j4 (actual arm angles)
    arm_joint_spped : array (1x4)
        Control velocities, to simulate the robot motion.
    delta_t : float
        difference between actual theta and past theta.
    arm_speed_limits : matrix (2x1)
        Matrix that describe the minimum and maximum speed allowed limits.
        [[min_arm_speed],[max_arm_speed]]
    Returns
    -------
    robot_state : array (1x5)
        Actual state of the robot (actual configuration)
        [j1,j2,j3,j4,suction_pad]
        j1...j4 (arm joint angles)
        suction_pad (0 for disable and 1 for enable)
    """
    new_arm_angles = []
    
    for i in range(0,len(arm_angles)):
        new_arm_angles.append(euler_step(arm_angles[i], 
                                         arm_joint_speed[i], 
                                         arm_speed_limits, 
                                         delta_t))
    
    return new_arm_angles


def robot_jacobian (theta_arm_list): 
    """
    Parameters
    ----------
    theta_arm_list : array (1x5)
        Actual arm joints theta angles in form [j1,j2,j3.j4].
        
    Returns
    -------
    Je : matrix (6x4)
        Jacobian representation of the arm_jacobian

    """
    arm_jacobian = mr.JacobianBody(config.B_list,theta_arm_list)
    
    return arm_jacobian




