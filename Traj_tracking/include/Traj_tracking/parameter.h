#pragma once

#define pi_1 3.1415926
#define EARTH_RADIUS 6378.137

//LQR parameter
#define LQR_Q11  0.005
#define LQR_Q22  0
#define LQR_Q33  1
#define LQR_Q44  0
#define LQR_R  1
#define LQR_iterative  50
#define Q11_ratio 1
#define Q33_ratio 1

//MPC parameter
#define MPC_Q11  0.005
#define MPC_Q22  0
#define MPC_Q33  1
#define MPC_Q44  0
#define MPC_R  1
#define MPC_Horizen  50//100

//vehicle parameter
#define veh_Mass  1670
#define veh_Iz  3084
#define veh_Cf  40207 * 2
#define veh_Cr  40207 * 2
#define veh_Lf  1.204
#define veh_Lr  1.286
#define veh_WheelBase 2.49
#define MaxTireAngle  0.705 //40.4 * pi_1 / 180
#define SteerRatio 10.47

#define discrete_Ts  0.01

#define window_size 10