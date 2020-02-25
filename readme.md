## Obstacle Avoidance of Autonomous Mobile Robots using Model Predictive Control

This seminar investigates obstacle avoidance of autonomous mobile robots (AMRs) in an indoor environment where aerodynamic side forces and gravitational side forces can be neglected. Emphasis is lain on lateral displacement control, for this reason, a constant longitudinal velocity is assumed, and perfect trajectory tracking along the longitudinal axis is assumed.


A configuration space of 50m by 50m is considered with obstacles scatered around this space. The obstacles in this study are assumed to be static and of rounded geometry of radius 1m. It is also assumed that their positions are known.


Path planning and trajectory generation are essential components of any obstacle avoidance scheme. In this study, these two important elements are provided using the concept of artificial potential fields (APFs).


--------------------------------------------------------

### PATH PLANNING AND TRAJECTORY GENERATION

The concept of path planning using APFs was first proposed by Oussama Khatib in 1986. Since then numerous studies have been carried out to investigate and further enhance this technique. While there are numerous problems associated with path planning using APFs, it is a very good concept to study as it provides intuition into the path planning process.

An attractive potential is setup such that its minimum is at the goal position, while a repulsive potential is setup at and around the position of each obstacle. These two potentials are superimposed and a universal potential field is produced, along which the AMR is guided to its goal position.

The attractive potential used in this study is given as:

![equation](https://latex.codecogs.com/gif.latex?U_%7BA%7D%3A%3D%20%5Cfrac%7B1%7D%7B2%7D%20K_%7Batt%7D%20%7B%5Cleft%20%5C%7C%20%5Cvec%7BX%7D%20-%20%5Cvec%7BX_%7Bgoal%7D%7D%20%5Cright%20%5C%7C%7D%5E%7B2%7D)

The repulsive potential is given as:

![equation](https://latex.codecogs.com/gif.latex?U_%7BR%7D%3A%3D%20%5Cfrac%7B1%7D%7B2%7D%20K_%7Brep%7D%20%5Cleft%28%20%7B%5Cfrac%7B1%7D%7B%5Cleft%20%5C%7C%20%5Cvec%7BX%7D%20-%20%5Cvec%7BX_%7Bobs%7D%7D%20%5Cright%20%5C%7C%7D%7D%20-%20%5Cfrac%7B1%7D%7Bp_0%7D%20%5Cright%20%29%5E%7B2%7D)

where: 

*K_att*, *K_rep*, and *p_0* are constants.

*X* is the current position, *X_goal* the goal position, and *X_obs* the position of the current obstacle (note that the repulsive potential is formulated for each known obstacle).

The universal potential field is generated as:

![equation](https://latex.codecogs.com/gif.latex?U%3A%3D%20U_%7BA%7D%20&plus;%20U_%7BR%7D)


In this study, gradient descent approach is used to guide the AMR along the universal potential to its goal position while avoiding the obestacles in its way. The driving force (negative gradient) of the AMR towards its goal position resulting from the attractive potential is given as:

![equation](https://latex.codecogs.com/gif.latex?F_%7BA%7D%3A%3D%20-grad%28U_%7BA%7D%29)

![equation](https://latex.codecogs.com/gif.latex?F_%7BA%7D%3A%3D%20-%7BK_%7Batt%7D%20%5Cleft%28%20%5Cvec%7BX%7D%20-%20%5Cvec%7BX_%7Bgoal%7D%7D%20%5Cright%20%29%7D)

Following the same pattern, the driving force propelling the AMR away from obstacles modelled by the repulsive potential is given as:

![equation](https://latex.codecogs.com/gif.latex?F_%7BR%7D%3A%3D%20-grad%28U_%7BR%7D%29)

![equation](https://latex.codecogs.com/gif.latex?F_%7BR%7D%3A%3D%20%7BK_%7Brep%7D%20%5Cleft%28%20%5Cfrac%7B1%7D%7B%5Cleft%20%5C%7C%20%5Cvec%7BX%7D%20-%20%5Cvec%7BX_%7Bobs%7D%7D%5Cright%20%5C%7C%7D%20-%20%5Cfrac%7B1%7D%7Bp_0%7D%20%5Cright%20%29%20%5Cfrac%7B1%7D%7B%7B%5Cleft%20%5C%7C%20%5Cvec%7BX%7D%20-%20%5Cvec%7BX_%7Bobs%7D%7D%20%5Cright%20%5C%7C%7D%5E%7B3%7D%7D%20%5Cleft%28%20%5Cvec%7BX%7D%20-%20%5Cvec%7BX_%7Bobs%7D%7D%20%5Cright%20%29%7D)


The output of the path planning and trajectory generation algorithm is a trajectory with longitudinal and lateral positions via which the AMR can get to its goal position while avoiding the obstacles in its path. A trajectory of the yaw angle the AMR should assume is also calculated from the velocity vector at every time instance.

#### Associated Scripts
- trajectory_generator_apf.m
- trajectory_generator_apf_with_plot.m


--------------------------------------------------------

### MODEL

The AMR was modelled using the linearized lateral 2 DOF bicycle model of a vehicle. While this model has been extensively streamlined, it still provides valuable insight into the lateral dynamics of a vehicle. 

A small AMR with a single front wheel and 2 rear wheel is considered. More information about the considered model is given below:
- Mass, m: 505kg
- Yaw moment of inertia, I_zz: 808.5kg.m^2
- Front tyre cornering stiffness, C_f: 40kN/rad
- Rear tyre cornering stiffness, C_r: 40kN/rad
- Distance of front wheel from center of mass, a: 0.35m
- Distance of rear wheels from center of mass, b: 0.4125m

The state space representation of the model is given as:

![equation](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20%5Cdot%7Bx_c%7D%20%5C%5C%20%5Cdot%7B%5Cbeta%7D%20%5C%5C%20%5Cdot%7B%5Cpsi%7D%20%5C%5C%20%5Cddot%7B%5Cpsi%7D%20%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%200%20%26%20v%20%26%20v%20%26%200%20%5C%5C%200%20%26%20-%5Cfrac%7BC_f&plus;2C_r%7D%7Bmv%7D%20%26%200%20%26%20%7B%5Cfrac%7B2bC_r%20-%20aC_f%7D%7Bm%7Bv%7D%5E%7B2%7D%7D%20-%201%7D%20%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5C%5C%200%20%26%20%5Cfrac%7B2bC_r%20-%20aC_f%7D%7BI_%7Bzz%7D%7D%20%26%200%20%26%20-%5Cfrac%7B2%7Bb%7D%5E%7B2%7DC_r%20&plus;%20%7Ba%7D%5E%7B2%7DC_f%7D%7BI_%7Bzz%7Dv%7D%20%5Cend%7Bbmatrix%7D%20%5Cbegin%7Bbmatrix%7D%20x_c%20%5C%5C%20%5Cbeta%20%5C%5C%20%5Cpsi%20%5C%5C%20%5Cpsi%20%5Cend%7Bbmatrix%7D%20&plus;%20%5Cbegin%7Bbmatrix%7D%200%20%5C%5C%20%5Cfrac%7BC_f%7D%7Bmv%7D%20%5C%5C%200%20%5C%5C%20%5Cfrac%7BaC_f%7D%7BI_%7Bzz%7D%7D%20%5Cend%7Bbmatrix%7D%20%5Cdelta)

![equation](https://latex.codecogs.com/gif.latex?%5C%5Cx_c%3A%20lateral%5C%20displacement%5C%5C%20%5Cbeta%3A%20side%5C%20slip%5C%20angle%5C%5C%20%5Cpsi%3A%20yaw%5C%20angle%5C%5C%20%5Cdot%7B%5Cpsi%7D%3A%20yaw%5C%20rate%20%5C%5C%5C%5C%20%5Cdelta%3A%20front%5C%20wheel%5C%20angle)

Two variants of the model are specified, the first is a SISO model with the front wheel angle as the input and the lateral displacement of center of mass as the output (lateral_2_dof_model.m). The second variant has the yaw angle as an additional output, effectively making one input and 2 outputs (yaw_lateral_2_dof_model.m).

#### Associated Scripts
- lateral_2_dof_model.m
- yaw_lateral_2_dof_model.m


--------------------------------------------------------

### CONTROLLER DESIGN

Three different controllers were designed during the course of this study. While the main focus is on model predictive control (MPC), a PID controller was also designed as juxtaposing mechanism. Two MPC controllers were designed, one using the SISO model and the other using the SIMO model of the AMR.

#### PID Controller
The PID controller was designed using MATLAB's pidTuner tool. This model can be viewed in *pid_control_sim.slx*.


#### MPC Controller Design
Model predictive control has risen in popularity in both research and industry in recent years. This can mainly be attributed to rapid advances in computing technologies. Of all its pros, MPC was considered in this study largely because it allows embedding model constraints in the controller. This made it possible to specify rate and limit constraints on the control input (front wheel steering angle). The SIMO-based MPC controller has an equally weighted output (i.e. deviations from the reference the lateral displacement has an equally weighted effect on the optimization cost function as deviations from the reference yaw angle). 

Both MPC controllers are preset with a prediction horizon of 25 time steps and a control horizon of 4 time steps on a sampling time of 0.05 seconds.

**CONSTRAINTS:**

1. Maximum front wheel angle: 40 degrees  
1. Minimum front wheel angle: -40 degrees  
1. Maximum front wheel angular velocity: 30 degrees/second  

The MPCs were designed using the [MPC Controller](https://www.mathworks.com/help/releases/R2019b/mpc/ref/mpccontroller.html) model from Simulink's Model Predictive Control Toolbox.    

#### Associated Scripts
- pid_control.m
- mpc_control.m


--------------------------------------------------------

### SIMULATION

Extensive simulations were carried out on the effects of the designed controllers on a non-linear vehicle model. The [Vehicle Body 3DOF Single Track](https://www.mathworks.com/help/releases/R2019b/vdynblks/ref/vehiclebody3dof.html) model from Simulink's Vehicle Dynamics Blockset was used for simulations. In all simulation scenarios, perfect tracking of the AMR's longitudinal position is assumed since this study focuses on lateral displacement control. Hence, the refrence signal is the lateral position trajectory produced by the APF path planning algorithm. In addition to the lateral position reference, the yaw angle reference is also provided as reference for the SIMO-based MPC. 

Performance of the controllers are compared on the basis of a scaled error norm which is the *L2* norm of the error at each sampling instance divided by the number of samples.

#### SCENARIO 1
In this simulation scenario, a goal position of (50, 31) was set and obstacles were placed at (14.87, 33.28),  (10, 8), (26, 12), (19, 19), and (34, 23). The trajectory generator generated a suitable trajectory that avoids all known obstacles and assumes the AMR is at an initial position of (0, 0). The two MPCs were simulated with a sampling time of 0.05 seconds, while the PID controller was simulated with a sampling time of 0.01 seconds.  

In this scenario, the PID controller performed best, with a negligible scaled error norm of 0.0022. Next in line, although by a very thin margin is the SIMO-based MPC controller with a scaled error norm of 0.3647. And finally, comes the SISO-based MPC with an error norm of 0.4713. All three controllers provided control signals which are physically realizable. 

#### SCENARIO 2
The same goal position and obstacle positions from scenario 1 are used. However, the initial position of the AMR was offset by an error, such that the new initial position was (0, 1) while the path planning algorithm still planed a trajectory that assumed an initial position of (0, 0). The path planning parameters were tweaked a little, such that the region of influence of the obstacles were increased.  

Like scenario 1, the PID controller had a lower scaled error norm in this scenario also (about 0.0579). Under the influence of the PID controller, the AMR quickly converged to and strictly followed the reference signal. However, the control signals it generated look impossible to implement in a real life situation. It produced wheel angles with magnitude as high as 120 radians.

The SISO-based MPC could not successfully control the AMR to track the reference in this scenario. It produced a scaled error norm of about 14.55. *The perfect longitudinal displacement tracking was ommited for this particular case and the actual longitudinal output of the simulation model was used such that the movement of the AMR in circles is evident*. However, by increasing the prediction horizon to 35 time steps (from 25), the SISO MPC was able to control the AMR to converge to, and track the reference with a scaled error norm of 0.7093.  

#### SCENARIO 3
The same goal point and obstacle positions from the 2 previous scenarios are retained. Also, the initial position of the AMR is put back at (0, 0). The path planning parameters are modified to further increase the region of influence of the obstacles and change the course of the AMR's travel.  

The PID controller performed quite well, yielding a scaled error norm of about 0.0044. However, the control signal generated cannot be physically realized. The required front wheel angle attained a magnitude of about 5 radians at some point.  

The SIMO-based MPC was able to track the planned path with a scaled error norm of 0.3980. And as expected with an MPC with output constraints, the control signal produced was within a physically realizable limit.

The SISO-based MPC albeit with a larger scaled error norm of about 0.4938, was also able to take the AMR through the planned path with physically realizable control output.


#### Associated Scripts
- init_scen_1.m
- init_scene_2.m
- init_scene_3.m
- mpc_control.m
- pid_control.m  

Images related to the scenarios can be found in the */report/img* folder. Images labelled with *pid* belong to the PID controller; *mpc1* to the SISO-based MPC; and *mpc2* to the SIMO-based MPC. Images labelled with *apf* show the universal artificial potential field used for path planning.     