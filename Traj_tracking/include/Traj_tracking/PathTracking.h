#pragma once
#include "parameter.h"
#include "pid_reg3.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <qpOASES.hpp>
#include "TrajectoryAnalyse.h"
#include "Traj_tracking/ivsensorgps.h"
#include "Traj_tracking/u_desire.h"

USING_NAMESPACE_QPOASES
using Eigen::MatrixXd;

struct LateralMatrix {
	MatrixXd matrix_State ;
	MatrixXd matrix_A ;
	MatrixXd matrix_A_vary ;
	MatrixXd matrix_Ad ;
	MatrixXd matrix_B ;
	MatrixXd matrix_Bd ;
	MatrixXd matrix_GPR ;
	MatrixXd matrix_Q ;
	MatrixXd matrix_R;
	MatrixXd matrix_K;

    MatrixXd matrix_C;
    MatrixXd matrix_Cd;
	MatrixXd matrix_AA;
	MatrixXd matrix_AB;
	MatrixXd matrix_AC;
    MatrixXd matrix_Q_mpc;
    MatrixXd matrix_R_mpc;
	MatrixXd matrix_QQ;
    MatrixXd matrix_RR;
    MatrixXd matrix_M;
    MatrixXd matrix_H;
    MatrixXd matrix_f;
    MatrixXd matrix_lb;
    MatrixXd matrix_ub;
};

struct LongitudinalState {
	double u_desire;
	double velocity;
	int32_t throttle;
	int32_t brake;
};

class Lateral_Controller {
public:
	double tire_angle_feedback;
	double tire_angle_feedforward;
	double tire_angle;
	double steer_angle;
	
	PointType Curr_State;
	LateralMatrix lat_matrix;

	std::vector<double>lateral_error_set;
	std::vector<double>heading_error_set;

	void InitParam();
	void InitMatrix();
	void Upadte_Discrete_A(double);
	void ComputeStateMatrix(input_trajType,double);
	double StateFilter(std::vector<double>&);
	void SloveDiscreteLQR();
	void ComputeFeedforward(input_trajType);
	void ComputeFeedforward_GPR();
	void Constraint_Transform();

    void Callback_gps(const Traj_tracking::ivsensorgps::ConstPtr& msg);

    void MPC_InitMatrix();
    void MPC_Update_C();
    void MPC_Update_Discrete_AC(double);
    void MPC_Update_Matrix();
    void MPC_QPSolver();
};

class Longitudinal_Controller {
public:

    enum StateName {RESET, STARTUP, LOW, ACC, ADJUST, BRAKE, STABLE};
    StateName states;

	double m_u0;
	double u_desire;
	double kmph;
	int32_t throttle;
	int32_t brake;
    int brake_f;

    LongitudinalState Curr_LongState;
	LongitudinalState Last_LongState;

	PIDREG3 m_pidLongitudinal;
	PIDREG3 m_pidThrottle;
	PIDREG3 m_pidBrake;
	
	void InitParam();
	void Longitudinal();
    void Longitudinal_CAS();
	double desiredSpeedFilter();
	double desiredSpeedFilter2();

	void Callback_DesireVelocity(const Traj_tracking::u_desire::ConstPtr& msg);
    void Callback_LongVelocity(const Traj_tracking::ivsensorgps::ConstPtr& msg);
};
