# Animation
<p align="center">
 <img src="https://github.com/javierandrango/RobotSimulation/blob/scara/images/scara-csv.gif" width='30%'>
</p>


# Description
Motion control of a Scara robot (RRPR) with suction pad using velocity inputs.

Assuming that we have direct control over the joint velocities (for example if we use stepper motors as actuators). The motion control task can be expressed as a `task space`, where the controller is fed a steady stream of end-effector configurations $X_d(t)$, and the goal is to command joint velocities that cause the robot to track this trajectory. 

In simple words, we have to create a trajectory from point A to point B, then command the end-effector of the robot to track this trajactory.

The robot controller is described as `Feedforward plus P.I Feedback control`. The `feedforward control` is the simplest type of control where the commanded velocity($ \dot{\theta}(t)$) is equal to the desired joint trajectory($\dot{\theta}_d(t)$).

$$ \dot{\theta}(t) = \dot{\theta}_d(t)$$

In practice, positions errors will accumulate over time under the `feedforward control` law. An alternative strategy is to measure the actual position of each joint continually and limit the accumulation of errors is implement a `Proportional-Integral feedback control`

$$ \dot{\theta}(t) = \dot{\theta}_d(t) + K_p \theta_e(t) + k_i \int_{0}^{t} \theta_e(t)dt$$

<p align="center">
 <img src="https://github.com/javierandrango/RobotSimulation/blob/scara/images/feedforwardPI_plus_feedback_control.png" width='50%'>
</p>



`Feedforward plus P.I Feedback control` expresed as a task-space motion control:

$$V(t) = [Ad_{X^{-1}X_d}]V_d(t)+K_pX_{err}(t)+K_i \int_{0}^{t}X_{err}(t)dt,~~~~~, a~ 6x1~matrix$$

$$[V_d(t)] = \frac{1}{\Delta(t)}.log(X^{-1}_d . X_{d\_next})~~~~~, element~of~a~so(3)$$

$$[X_{err}]=log(X^{-1}X_d)~~~~~, element~of~a~so(3)$$

`Commanded joint velocities`:

$$\mu = Je^{\psi}(\theta)V$$
The commanded speed of the joints are equal to the dot product of the pseudo-inverse of the body Jacobian $\epsilon ~~\mathbb{R}^{6xn}$ where $n=4$ joints (6x4 matrix), and the $V$ term of the `Feedforward plus P.I Feedback control`(6x1 matrix).

# Prerequisites 
1. Understanding of `PoE` (product of exponentials) method for mapping the links of a kimematics chain in space. A detailed explanation is described in `Modern Robotics` book, chapter 1-6. A pre-printed version of the book can be found at: 
<a href="https://hades.mech.northwestern.edu/images/7/7f/MR.pdf" target="_blank">Modern Robotics: Mechanics, Planning and Control. By Kevin M. Lynch and Franck C. Park</a>

2. Undestanding of the robot controller theory and implementation. A detailed explanation is described in `Modern Robotics` book, chapter 11.3.

3. Understanding of the `ModernRobotics` Python package. this simplify calculations and code implementation. A detailed explanation is described in <a href="https://github.com/NxRLab/ModernRobotics" target="_blank">Modern Robotics: Code Library</a>

4. Download and installation of `CoppeliaSim Edu`. the program can be found <a href="https://www.coppeliarobotics.com/" target="_blank">Here</a>
 

# Usage
1. clone this repository
2. Open `CoppeliaSim`
3. Go to File>Open Scene..., and look for the `scara.ttt` file. the File contains the animation of the scara robot.
4. Start the simulation:
    - `CSV Playback` tab: Copy the directory of the `~\results\arm_motion.csv` file and paste into the big input area, then click on the `Open File` button, if the file is correct or incorrect a message will apear in the messages section. If the file is loaded click the `Play` button to start simulation.
    - `Joints` tab: Move the sliders to control the robot joints and the suction pad.
5. Click the `Stop` button to close the simulation.       
    

# Notes
> [!NOTE]
> Notes here


# Results
Trajectory:
<p align="center">
 <img src="https://github.com/javierandrango/RobotSimulation/blob/scara/images/trajectory.gif" width='50%'>
</p>

Robot control:
<p align="center">
 <img src="https://github.com/javierandrango/RobotSimulation/blob/scara/images/scara-csv.gif" width='50%'>
</p>

Manual control:
<p align="center">
 <img src="https://github.com/javierandrango/RobotSimulation/blob/scara/images/scara-joints.gif" width='50%'>
</p>




