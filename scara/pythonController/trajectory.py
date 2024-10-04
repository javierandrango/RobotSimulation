# -*- coding: utf-8 -*-
"""
@author: Javier
"""
import modern_robotics as mr
import matplotlib.pyplot as plt
import numpy as np
import math as math
import configuration as config
from pytransform3d.rotations import plot_basis
import matplotlib.animation as animation

def trajectory_steps(Xstart,Xend,suctionpad,Tf,N,method):
    """
    Parameters
    ----------
    Xstart : Matrix 4x4
        The initial end-effector configuration.
    Xend : Matrix 4x4
        The final end-effector configuration.
    suctionpad: int
        Actual state of the gripper (0 for open and 1 for close state).
    Tf : float
        Total time of the motion Tf in seconds from rest to rest.
    N : int
        The number of points N ≥ 2 in the discrete representation of the trajectory.
    method : int
        3 indicates cubic (third-order polynomial) time scaling.
        5 indicates quintic (fifth-order polynomial) time scaling.  

    Returns
    -------
    trajectory : matrix Nx13.
        The discretized trajectory in form of:
        [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,suctionpad]    
        The first row is Xstart plus suction pad state and the Nth is Xend plus 
        suction pad state.
    """
    list_matrix_trajectory = mr.ScrewTrajectory(Xstart,Xend,Tf,N,method)
    list_len = len(list_matrix_trajectory)
    trajectory = np.empty((list_len,13),float)
    
    for  i in range(0,list_len):
        trajectory[i,0:9]= list_matrix_trajectory[i][0:3,0:3].flatten()
        trajectory[i,9:12]= list_matrix_trajectory[i][0:3,-1].flatten()
        trajectory[i,-1]= suctionpad

    return trajectory

def suctionpad_steps(last_trajectory_row,Tf,k,suctionpad_state):
    """
    Parameters
    ----------
    last_trajectory_row : row 1x13
        last row from previous matrix trajectory.
    Tf : int
        Total time of the motion Tf in seconds from rest to rest.
    k : int
        The number of trajectory reference configurations per 0.01 seconds.
    suctionpad_state: int
        Actual state of the suction pad (0 for disable and 1 for enable).
    Returns
    -------
    suctionpad_trajectory : matrix Nx13
        The discretized trajectory in form of:
        [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,gripper]
        "ONLY CHANGE THE GRIPPER STATE" and keeps the trajectory of the arm     
    """
    N = round(Tf*k/0.01)
    suctionpad_trajectory =np.empty((N,13),float)
    for i in range (0,N):
        suctionpad_trajectory[i,0:12] = last_trajectory_row[0:-1]
        suctionpad_trajectory[i,-1] = suctionpad_state
     
    return suctionpad_trajectory

def point_to_point_speed_approx (init_configuration,end_configuration, max_velocity, k):
    """
    Parameters
    ----------
    init_configuration : matrix 4x4
        initial trajectory configuration.
    end_configuration : matrix 4x4
        final trajectory configuration.
    max_velocity : float
        linear maximun velocity for the end effector.
    k : int
        The number of trajectory reference configurations per 0.01 seconds.

    Returns
    -------
    Tf : float
        Total time of the motion Tf in seconds from rest to rest.
    N : int
        The number of points N ≥ 2 in the discrete representation of the 
        trajectory.

    """
    p2 = end_configuration[0:3,-1]
    p1 = init_configuration[0:3,-1]
    distance = math.sqrt(((p2[0]-p1[0])**2) + ((p2[1]-p1[1])**2) + ((p2[2]-p1[2])**2))
    Tf = round(distance/max_velocity,2)
    N = round(Tf*k/0.01)
    return  Tf,N

def rot_z(z_rad):
    """
    Parameters
    ----------
    z_rad: float
        Angle in radians.
    Returns
    -------
    Rz: Matrix 3x3
        Rotation matriz along the z-axis
    """
    Rz = np.array([[math.cos(z_rad), -math.sin(z_rad), 0],
                   [math.sin(z_rad), math.cos(z_rad),  0],
                   [0,               0,                1]])
    return Rz

def animate_frame(frame,trajectory,step,ax):
    """
    Parameters
    ----------
    frame : int
        actual frame of the animation.
    trajectory : Matrix Nx13
        The discretized trajectory in form of:
        [r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,suctionpad]    
        The first row is Xstart plus suction pad state and the Nth is Xend plus 
        suction pad state.
    step : int 
        Step of a secuence.
    ax : TYPE
        DESCRIPTION.

    Returns
    -------
    ax : mplot3d.axes3d.Axes3D
        Subplot.

    """
    # Set axis limits and labels again since we cleared the plot
    ax.cla()
    ax.set_xlim((-2, 2))
    ax.set_ylim((-2, 2))
    ax.set_zlim((-1, 1))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    
    if step <= 0:
        step = 1
    
    trajectory_steps = trajectory[[i for i in range(0,len(trajectory),step)],:]
    
    for i in range(len(trajectory_steps)):
        ax.scatter(trajectory_steps[i,9],
                   trajectory_steps[i,10],
                   trajectory_steps[i,11],
                   color='gray')
    
    R = trajectory_steps[frame,0:9].reshape(3,3)
    p = trajectory_steps[frame,9:12]
    plot_basis(ax=ax, s=0.5, R=R, p=p)
    return ax


if __name__ == "__main__":
    max_velocity = 0.25 # max linear velocity in m/s for the end-effector 
    k = 1 # number of trajectory reference configuration per 0.01 seconds
    Tsb_init = config.M
    Tsb_standoff_init = mr.RpToTrans(rot_z(0), np.array([1.73455,0,0.2]))
    Tsb_rot = mr.RpToTrans(rot_z(-math.pi/4), np.array([1.73455,0,0.364]))
    Tsb_final_offset =  mr.RpToTrans(rot_z(0), np.array([-0.715,1.225,0.364]))
    Tsb_standoff_final =  mr.RpToTrans(rot_z(0), np.array([-0.715,1.225,0.2]))
    
    #end effector
    # move to standoff initial configuration
    Tf1,N1 = point_to_point_speed_approx (Tsb_init,Tsb_standoff_init, max_velocity, k)
    Traj1=trajectory_steps(Tsb_init,Tsb_standoff_init,0,Tf1,N1,3)
    # activate suction pad
    Traj2=suctionpad_steps(Traj1[-1,:], 0.02, 1, 1)
    # move to initial position and rotate -45deg
    Tf3,N3 = point_to_point_speed_approx (Tsb_standoff_init,Tsb_rot, max_velocity, k)
    Traj3=trajectory_steps(Tsb_standoff_init,Tsb_rot,1,Tf1,N1,3)
    # move to final z-offset configuration
    Tf4,N4 = point_to_point_speed_approx (Tsb_rot,Tsb_final_offset, max_velocity, k)
    Traj4=trajectory_steps(Tsb_rot,Tsb_final_offset,1,Tf1,N1,3)
    # move to standoff final configuration
    Tf5,N5 = point_to_point_speed_approx (Tsb_final_offset,Tsb_standoff_final, max_velocity, k)
    Traj5=trajectory_steps(Tsb_final_offset,Tsb_standoff_final,1,Tf1,N1,3)
    # deactivate suction pad
    Traj6=suctionpad_steps(Traj5[-1,:], 0.02, 1, 0)
    # move to final z-offset configuration
    Tf7,N7 = point_to_point_speed_approx (Tsb_standoff_final,Tsb_final_offset, max_velocity, k)
    Traj7=trajectory_steps(Tsb_standoff_final,Tsb_final_offset,0,Tf1,N1,3)
    #move to initial offset position
    Tf8,N8 = point_to_point_speed_approx (Tsb_final_offset,Tsb_init, max_velocity, k)
    Traj8=trajectory_steps(Tsb_final_offset,Tsb_init,0,Tf1,N1,3)

    complete_trajectory = np.concatenate((Traj1,Traj2,Traj3,Traj4,Traj5,Traj6,Traj7,Traj8))
    
    # save screw trajectory as csv file
    config.save_file(complete_trajectory,"screw_trajectory")
    
    # animation
    # Prepare the figure and axes
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim((-2, 2))
    ax.set_ylim((-2, 2))
    ax.set_zlim((-1, 1))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    step = 5
    n_frames = int(len(complete_trajectory)/step)
    anim = animation.FuncAnimation(fig, animate_frame, frames=n_frames, fargs=(complete_trajectory,step,ax), interval=10, blit=False)
    # Show the animation
    plt.show()
    
    
    