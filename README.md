# LBOptimalControl
Fast Learning-based optimal controller for autonomous driving
the file "Traj_tracking" is a Node under ROS (kinetic) 

The whole code in programmed for the control algorithm "Fast learning\kernel-based optimal control" in real application.
Mainly contains three parts, which are modeling, control scheme and Gauusian process regression(GPR).
The modeling part contains dynamic model and residual GP model, where the residual model is derived by a supervised learning algorithm (GPR) from data set. 
The lateral control algorithm consists of optimal control scheme and the corresponding MPC algorithm for comparison, and digital PID with auti-windup algorithm is used for velocity-tracking.
