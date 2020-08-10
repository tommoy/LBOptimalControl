#include "Traj_tracking/PathTracking.h"
#include "Traj_tracking/TrajectoryAnalyse.h"

void Lateral_Controller::InitParam() {
    tire_angle_feedback = 0;
    tire_angle_feedforward = 0;
    tire_angle = 0;
    steer_angle = 0;
}
void Lateral_Controller::InitMatrix() {

    lat_matrix.matrix_State = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_A = MatrixXd::Zero(4, 4);
    lat_matrix.matrix_A_vary = MatrixXd::Zero(4, 4);
    lat_matrix.matrix_Ad = MatrixXd::Zero(4, 4);
    lat_matrix.matrix_B = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_Bd = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_GPR = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_Q = MatrixXd::Identity(4, 4);
    lat_matrix.matrix_K = MatrixXd::Zero(1, 4);
    lat_matrix.matrix_R = MatrixXd::Zero(1, 1);

    lat_matrix.matrix_A(0, 1) = 1.0;
    lat_matrix.matrix_A(1, 2) = (veh_Cf + veh_Cr) / veh_Mass;
    lat_matrix.matrix_A(2, 3) = 1.0;
    lat_matrix.matrix_A(3, 2) = (veh_Lf * veh_Cf - veh_Lr * veh_Lr) / veh_Iz;

    lat_matrix.matrix_A_vary(1, 1) = -1.0 * (veh_Cf + veh_Cr) / veh_Mass;
    lat_matrix.matrix_A_vary(1, 3) = (veh_Lr * veh_Cr - veh_Lf * veh_Cf) / veh_Mass;
    lat_matrix.matrix_A_vary(3, 1) = (veh_Lr * veh_Cr - veh_Lf * veh_Cf / veh_Iz);
    lat_matrix.matrix_A_vary(3, 3) = -1.0 * (veh_Lf * veh_Lf * veh_Cf + veh_Lr * veh_Lr * veh_Cr) / veh_Iz;

    lat_matrix.matrix_B(1, 0) = veh_Cf / veh_Mass;
    lat_matrix.matrix_B(3, 0) = veh_Lf * veh_Cf / veh_Iz;
    lat_matrix.matrix_Bd(1, 0) = discrete_Ts * veh_Cf / veh_Mass;
    lat_matrix.matrix_Bd(3, 0) = discrete_Ts * veh_Lf * veh_Cf / veh_Iz;

    lat_matrix.matrix_Q(0, 0) = LQR_Q11 * Q11_ratio;
    lat_matrix.matrix_Q(1, 1) = LQR_Q22;
    lat_matrix.matrix_Q(2, 2) = LQR_Q33 * Q33_ratio;
    lat_matrix.matrix_Q(3, 3) = LQR_Q44;
    lat_matrix.matrix_R(0) = LQR_R;
}
void Lateral_Controller::Callback_gps(const Traj_tracking::ivsensorgps::ConstPtr& msg){
    Curr_State.lng = msg->lon;
    Curr_State.lat = msg->lat;
    Curr_State.head = msg->heading;
    Curr_State.velocity = msg->velocity;
    Curr_State.yawrate = msg->yaw * pi_1/180;
//    std::cout<<"Get Current State!"<<std::endl;
}
void Lateral_Controller::Upadte_Discrete_A(double velocity) {
    lat_matrix.matrix_A(1, 1) = lat_matrix.matrix_A_vary(1, 1) / velocity;
    lat_matrix.matrix_A(1, 3) = lat_matrix.matrix_A_vary(1, 3) / velocity;
    lat_matrix.matrix_A(3, 1) = lat_matrix.matrix_A_vary(3, 1) / velocity;
    lat_matrix.matrix_A(3, 3) = lat_matrix.matrix_A_vary(3, 3) / velocity;

    MatrixXd matrix_I = MatrixXd::Identity(4, 4);
    lat_matrix.matrix_Ad = (matrix_I - discrete_Ts * 0.5 * lat_matrix.matrix_A).inverse() *
                           (matrix_I + discrete_Ts * 0.5 * lat_matrix.matrix_A);
}
void Lateral_Controller::ComputeStateMatrix(input_trajType target_traj,double velocity) {
    double dx = 6371004.0 * cos(Curr_State.lat * pi_1 / 180.0) * 2.0 * pi_1 / 
		360.0 * (Curr_State.lng - target_traj.lng);
    double dy = 6371004.0 * (Curr_State.lat - target_traj.lat) * pi_1 / 180.0;

    double lateral_error = cos(target_traj.head*pi_1/180) * dy - sin(target_traj.head*pi_1/180) * dx;
    double heading_error = (Curr_State.head - target_traj.head)*pi_1/180;
    double lateral_error_rate = Curr_State.velocity * sin(heading_error);
//    double heading_error_rate = Curr_State.yawrate - target_traj.cur * Curr_State.velocity;
	double heading_error_rate = Curr_State.yawrate - target_traj.cur * velocity;

    lateral_error_set.push_back(lateral_error);
    heading_error_set.push_back(heading_error);

    if(lateral_error_set.size() > window_size - 1){
        lateral_error = StateFilter(lateral_error_set);
        std::vector<double>::iterator it = lateral_error_set.begin();
        lateral_error_set.erase(it);
    }
    if(heading_error_set.size() > window_size - 1){
        heading_error = StateFilter(heading_error_set);
        std::vector<double>::iterator it = heading_error_set.begin();
        heading_error_set.erase(it);
    }

    lat_matrix.matrix_State(0) = lateral_error;
    lat_matrix.matrix_State(1) = lateral_error_rate;
    lat_matrix.matrix_State(2) = heading_error;
    lat_matrix.matrix_State(3) = heading_error_rate;
    std::cout<<"State: "<<lat_matrix.matrix_State<<std::endl;
}
double Lateral_Controller::StateFilter(std::vector<double>& Sequence) {
    auto Min_temp = *std::min_element(Sequence.begin(),Sequence.end());
    auto Max_temp = *std::max_element(Sequence.begin(),Sequence.end());
    double Sum_temp;
    std::vector<double>::iterator it;
    for (it = Sequence.begin(); it != Sequence.end(); it++) {
        Sum_temp += *it ;
    }
    double value_filter = (Sum_temp - Min_temp - Max_temp) / (window_size - 2);
    return value_filter;

}
void Lateral_Controller::SloveDiscreteLQR() {

    MatrixXd A = lat_matrix.matrix_Ad;
    MatrixXd B = lat_matrix.matrix_Bd;
    MatrixXd AT = lat_matrix.matrix_Ad.transpose();
    MatrixXd BT = lat_matrix.matrix_Bd.transpose();
    MatrixXd Q = lat_matrix.matrix_Q;
    MatrixXd R = lat_matrix.matrix_R;

    MatrixXd P = lat_matrix.matrix_Q;
    int iter = 0;
    while (iter < LQR_iterative){
        MatrixXd PNext = AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
        iter++;
        P = PNext;
    }
    lat_matrix.matrix_K = (R + BT * P * B).inverse() * BT * P * A;

    tire_angle_feedback = (-1.0 * lat_matrix.matrix_K * lat_matrix.matrix_State)(0, 0);
}
void Lateral_Controller::ComputeFeedforward(input_trajType target_traj) {
    double kv = veh_Lr * veh_Mass / 2 / veh_Cf / veh_WheelBase - veh_Lf * veh_Mass / 2 / veh_Cr / veh_WheelBase;
    double feedforward1 = (double)veh_WheelBase * target_traj.cur;
    double feedforward2 = kv * Curr_State.velocity * Curr_State.velocity * target_traj.cur;
    double feedforward3 = veh_Lr * target_traj.cur - veh_Lf * veh_Mass * Curr_State.velocity* Curr_State.velocity * target_traj.cur / 2 / veh_Cr / veh_WheelBase;

    tire_angle_feedforward = feedforward1 + feedforward2 - lat_matrix.matrix_K(2)*feedforward3;

}
void Lateral_Controller::Constraint_Transform() {
    tire_angle = tire_angle_feedback + tire_angle_feedforward;
    if (tire_angle > MaxTireAngle) { tire_angle = MaxTireAngle; }
    if (tire_angle < -1.0*MaxTireAngle) { tire_angle = -1.0*MaxTireAngle; }
    //filter
    steer_angle = tire_angle * SteerRatio * 180/pi_1;
    //transform2PJLcan
}

void Lateral_Controller::MPC_InitMatrix() {
    lat_matrix.matrix_State = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_A = MatrixXd::Zero(4, 4);
    lat_matrix.matrix_A_vary = MatrixXd::Zero(4, 4);
    lat_matrix.matrix_Ad = MatrixXd::Zero(4, 4);
    lat_matrix.matrix_B = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_Bd = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_C = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_Cd = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_GPR = MatrixXd::Zero(4, 1);
    lat_matrix.matrix_Q_mpc = MatrixXd::Identity(4, 4);
    lat_matrix.matrix_K = MatrixXd::Zero(1, 4);
    lat_matrix.matrix_R_mpc = MatrixXd::Zero(1, 1);

    lat_matrix.matrix_A(0, 1) = 1.0;
    lat_matrix.matrix_A(1, 2) = (veh_Cf + veh_Cr) / veh_Mass;
    lat_matrix.matrix_A(2, 3) = 1.0;
    lat_matrix.matrix_A(3, 2) = (veh_Lf * veh_Cf - veh_Lr * veh_Lr) / veh_Iz;

    lat_matrix.matrix_A_vary(1, 1) = -1.0 * (veh_Cf + veh_Cr) / veh_Mass;
    lat_matrix.matrix_A_vary(1, 3) = (veh_Lr * veh_Cr - veh_Lf * veh_Cf) / veh_Mass;
    lat_matrix.matrix_A_vary(3, 1) = (veh_Lr * veh_Cr - veh_Lf * veh_Cf / veh_Iz);
    lat_matrix.matrix_A_vary(3, 3) = -1.0 * (veh_Lf * veh_Lf * veh_Cf + veh_Lr * veh_Lr * veh_Cr) / veh_Iz;

    lat_matrix.matrix_B(1, 0) = veh_Cf / veh_Mass;
    lat_matrix.matrix_B(3, 0) = veh_Lf * veh_Cf / veh_Iz;
    lat_matrix.matrix_Bd(1, 0) = discrete_Ts * veh_Cf / veh_Mass;
    lat_matrix.matrix_Bd(3, 0) = discrete_Ts * veh_Lf * veh_Cf / veh_Iz;

    lat_matrix.matrix_Q_mpc(0, 0) = MPC_Q11;
    lat_matrix.matrix_Q_mpc(1, 1) = MPC_Q22;
    lat_matrix.matrix_Q_mpc(2, 2) = MPC_Q33;
    lat_matrix.matrix_Q_mpc(3, 3) = MPC_Q44;
    lat_matrix.matrix_R_mpc(0) = MPC_R;

    lat_matrix.matrix_AA = MatrixXd::Zero(4*MPC_Horizen, 4);
    lat_matrix.matrix_AB = MatrixXd::Zero(4*MPC_Horizen, MPC_Horizen);
    lat_matrix.matrix_AC = MatrixXd::Zero(4*MPC_Horizen, 1);
    lat_matrix.matrix_QQ = MatrixXd::Zero(4*MPC_Horizen,4*MPC_Horizen);
    lat_matrix.matrix_M = MatrixXd::Zero(4*MPC_Horizen,1);
    lat_matrix.matrix_RR = MatrixXd::Identity(MPC_Horizen,MPC_Horizen);
    lat_matrix.matrix_lb = -1.0 * MaxTireAngle * MatrixXd::Ones(MPC_Horizen,1);
    lat_matrix.matrix_ub = 1.0 * MaxTireAngle * MatrixXd::Ones(MPC_Horizen,1);
    lat_matrix.matrix_H = MatrixXd::Zero(MPC_Horizen,MPC_Horizen);
    lat_matrix.matrix_f = MatrixXd::Zero(MPC_Horizen,1);
}
void Lateral_Controller::MPC_Update_Discrete_AC(double velocity) {
    lat_matrix.matrix_A(1, 1) = lat_matrix.matrix_A_vary(1, 1) / velocity;
    lat_matrix.matrix_A(1, 3) = lat_matrix.matrix_A_vary(1, 3) / velocity;
    lat_matrix.matrix_A(3, 1) = lat_matrix.matrix_A_vary(3, 1) / velocity;
    lat_matrix.matrix_A(3, 3) = lat_matrix.matrix_A_vary(3, 3) / velocity;
    MatrixXd matrix_I = MatrixXd::Identity(4, 4);
    lat_matrix.matrix_Ad = (matrix_I - discrete_Ts * 0.5 * lat_matrix.matrix_A).inverse() *
                           (matrix_I + discrete_Ts * 0.5 * lat_matrix.matrix_A);

    lat_matrix.matrix_C(1,0) = (veh_Cr*veh_Lr - veh_Cf*veh_Lf) / (veh_Mass*velocity) - velocity;
    lat_matrix.matrix_C(3,0) = -1.0*(veh_Cr*veh_Lr*veh_Lr - veh_Cf*veh_Lf*veh_Lf)/(veh_Iz*velocity);
    lat_matrix.matrix_Cd(1,0) = lat_matrix.matrix_C(1,0) * discrete_Ts;
    lat_matrix.matrix_Cd(3,0) = lat_matrix.matrix_C(3,0) * discrete_Ts;
}
void Lateral_Controller::MPC_Update_Matrix() {
    lat_matrix.matrix_AA.block(0,0,4,4)=lat_matrix.matrix_Ad;
    for(size_t i=1;i<MPC_Horizen;++i){
        lat_matrix.matrix_AA.block(i*4,0,4,4)=
                lat_matrix.matrix_Ad*lat_matrix.matrix_AA.block((i-1)*4,0,4,4);
    }

    std::vector<MatrixXd> matrix_A_temp(MPC_Horizen);
    matrix_A_temp[0]=lat_matrix.matrix_Ad;
    for(size_t i=1; i < MPC_Horizen; ++i){
        matrix_A_temp[i]=lat_matrix.matrix_Ad*matrix_A_temp[i-1];
    }
    lat_matrix.matrix_AB.block(0,0,4,1)=lat_matrix.matrix_Bd;
    for(size_t i=1; i < MPC_Horizen; ++i){
        for(size_t j=0;j<i;++j){
            lat_matrix.matrix_AB.block(4*i,j,4,1) = matrix_A_temp[i-j-1]*lat_matrix.matrix_Bd;
        }
        lat_matrix.matrix_AB.block(4*i,i,4,1)=lat_matrix.matrix_Bd;
    }

    lat_matrix.matrix_M.block(0,0,4,1) = lat_matrix.matrix_Ad*lat_matrix.matrix_State;
    for(size_t i=1; i < MPC_Horizen; ++i){
        lat_matrix.matrix_M.block(4*i,0,4,1)=
                lat_matrix.matrix_Ad*lat_matrix.matrix_M.block(4*(i-1),0,4,1);
    }

    for(size_t i=0; i < MPC_Horizen; ++i){
        lat_matrix.matrix_QQ.block(i*lat_matrix.matrix_Q_mpc.rows(),i*lat_matrix.matrix_Q_mpc.cols(),
                lat_matrix.matrix_Q_mpc.rows(),lat_matrix.matrix_Q_mpc.cols())=lat_matrix.matrix_Q_mpc;
        lat_matrix.matrix_RR.block(i*lat_matrix.matrix_R_mpc.rows(),i*lat_matrix.matrix_R_mpc.cols(),
                lat_matrix.matrix_R_mpc.rows(),lat_matrix.matrix_R_mpc.cols())=lat_matrix.matrix_R_mpc;
    }

    lat_matrix.matrix_AC.block(0,0,lat_matrix.matrix_Cd.rows(),1) = lat_matrix.matrix_Cd;
    for(size_t i=1; i < MPC_Horizen; ++i){
        lat_matrix.matrix_AC.block(i*lat_matrix.matrix_Cd.rows(),0,lat_matrix.matrix_Cd.rows(),1) =
                lat_matrix.matrix_AC.block((i-1)*lat_matrix.matrix_Cd.rows(),0,lat_matrix.matrix_Cd.rows(),1) +
                lat_matrix.matrix_AA.block((i-1)*lat_matrix.matrix_Ad.rows(),0,lat_matrix.matrix_Ad.rows(),
                        lat_matrix.matrix_Ad.cols())*lat_matrix.matrix_Cd;
    }

    lat_matrix.matrix_H = lat_matrix.matrix_AB.transpose() * lat_matrix.matrix_QQ * lat_matrix.matrix_AB + lat_matrix.matrix_RR;
    lat_matrix.matrix_f = lat_matrix.matrix_AB.transpose() * lat_matrix.matrix_QQ * (lat_matrix.matrix_M + lat_matrix.matrix_AC);
}
void Lateral_Controller::MPC_QPSolver() {
    real_t H[lat_matrix.matrix_H.rows() * lat_matrix.matrix_H.cols()];
    int index_H=0;
    for (size_t i = 0; i < lat_matrix.matrix_H.rows(); i++) {
        for (size_t j = 0; j < lat_matrix.matrix_H.cols(); j++) {
            H[index_H] = lat_matrix.matrix_H(i,j);
            index_H++;
        }
    }
    real_t f[lat_matrix.matrix_f.rows() * lat_matrix.matrix_f.cols()];
    int index_f=0;
    for (size_t i = 0; i < lat_matrix.matrix_f.rows(); i++) {
        for (size_t j = 0; j < lat_matrix.matrix_f.cols(); j++) {
            f[index_f] = lat_matrix.matrix_f(i,j);
            index_f++;
        }
    }
    real_t A[lat_matrix.matrix_H.rows()*lat_matrix.matrix_H.rows()];
    MatrixXd Eye_temp = MatrixXd::Identity(lat_matrix.matrix_H.rows(),lat_matrix.matrix_H.rows());
    int index_A=0;
    for (size_t i = 0; i < lat_matrix.matrix_lb.rows(); i++) {
        for (size_t j = 0; j < lat_matrix.matrix_lb.cols(); j++) {
            A[index_A] = Eye_temp(i,j);
            index_A++;
        }
    }
    real_t lb[lat_matrix.matrix_lb.rows() * lat_matrix.matrix_lb.cols()];
    int index_lb=0;
    for (size_t i = 0; i < lat_matrix.matrix_lb.rows(); i++) {
        for (size_t j = 0; j < lat_matrix.matrix_lb.cols(); j++) {
            lb[index_lb] = lat_matrix.matrix_lb(i,j);
            index_lb++;
        }
    }
    real_t ub[lat_matrix.matrix_ub.rows() * lat_matrix.matrix_ub.cols()];
    int index_ub=0;
    for (size_t i = 0; i < lat_matrix.matrix_ub.rows(); i++) {
        for (size_t j = 0; j < lat_matrix.matrix_ub.cols(); j++) {
            ub[index_ub] = lat_matrix.matrix_ub(i,j);
            index_ub++;
        }
    }

    QProblem example( lat_matrix.matrix_H.rows(),lat_matrix.matrix_lb.rows());
    int maxIter = 100;
//    example.init(H,f,A,NULL,NULL,lb,ub,maxIter);
    example.init(H,f,A,NULL,NULL,NULL,NULL,maxIter);
    real_t xOpt[lat_matrix.matrix_H.rows()];
    example.getPrimalSolution(xOpt);


    std::cout<<xOpt[0]<<std::endl;
}

void Longitudinal_Controller::InitParam() {
    //u_desire = 0;
    throttle = 0;
    brake = 0;
}
void Longitudinal_Controller::Longitudinal() {
    if (Curr_LongState.velocity < 0.5){
        //m_u0=0;
        Curr_LongState.throttle = 37;
        if (u_desire > 5)
            Curr_LongState.brake = -25500;
        if (m_pidThrottle.Out > 0)
            m_u0 = 0;
        m_pidThrottle.Out = 0;
    }
    else
    {
        int mode = 0;
        m_pidThrottle.Kp = 35;// 40
        m_pidThrottle.Ki = 0.005;
        m_pidThrottle.Kd = 0.04;
        m_pidThrottle.OutMax = 100;
        m_pidThrottle.OutMin = -10;
        m_pidThrottle.Fdb = Curr_LongState.velocity;
        //if ( u_desire -  speed*3.6 >5)
        //m_pidThrottle.Ref = desiredSpeedFilter()/ 3.6;
        //else

        if (u_desire > Last_LongState.u_desire || u_desire > Curr_LongState.velocity * 3.6){
            m_u0 = Curr_LongState.velocity * 3.6;
            if (m_u0 > u_desire)
                m_u0 = u_desire;
        }
        //if(0<u_desire-speed*3.6<1)
        //{
        // m_pidThrottle.Kp = 10;
        // m_pidThrottle.Ki=0;
        //}
        m_pidThrottle.Ref = desiredSpeedFilter()/3.6;

        pid_reg3_calc(&m_pidThrottle);
        if (m_pidThrottle.Ui > m_pidThrottle.OutMax) {
            m_pidThrottle.Ui = m_pidThrottle.OutMax;
        }

        if (m_pidThrottle.Out > 0){
            if (Last_LongState.throttle < 37)
                Last_LongState.throttle = 37;

            Curr_LongState.throttle = m_pidThrottle.Out + 37;
            if (Curr_LongState.throttle - Last_LongState.throttle > 6)
                Curr_LongState.throttle = Last_LongState.throttle + 6;
            //if ( pitch > 0)
            //{
            //pitch = 0;
            //}
            //double add = 90* speed * sin(pitch * PI / 180);
            //throttle += 90* speed * sin(pitch * PI / 180);
            Curr_LongState.brake = -25500;
            m_pidBrake.Ui = 0;
            m_pidBrake.Out = 0;
        }
        else{
            Curr_LongState.throttle = 37;
            m_pidThrottle.Ui = 0;
            m_pidThrottle.Out = 0;
            if (Last_LongState.brake > -50000)
                Last_LongState.brake = -50000;
            m_pidBrake.Kp = 10;
            m_pidBrake.Ki = 0.0012;
            m_pidBrake.Kd = 0.9;
            m_pidBrake.OutMax = 100;
            m_pidBrake.OutMin = 0;
            m_pidBrake.Ref = Curr_LongState.velocity * 3.6;
            //m_pidBrake.Ref = 0.1*(speed*3.6 - u_desire);
            //m_pidBrake.Fdb = u_desire;
            m_pidBrake.Fdb = desiredSpeedFilter2();
            pid_reg3_calc(&m_pidBrake);
            Curr_LongState.brake = -(200 * m_pidBrake.Out + 50000);

            if (Curr_LongState.brake - Last_LongState.brake <= -600)
                Curr_LongState.brake = Last_LongState.brake - 600;
        }
    }
    if (u_desire < 0.5)
    {
        double kmph = Curr_LongState.velocity * 3.6;
        if (kmph < 5)
            Curr_LongState.brake = -61600;
        else if (kmph < 10)
            Curr_LongState.brake = -62000;
        else if (kmph < 20)
            Curr_LongState.brake = -64000;
        else if (kmph < 30)
            Curr_LongState.brake = -66000;
        else if (kmph < 40)
            Curr_LongState.brake = -68000;
        else if (kmph < 50)
            Curr_LongState.brake = -69000;
        else
            Curr_LongState.brake = -70000;
        Curr_LongState.throttle = 37;
        m_pidThrottle.Ui = 0;
        m_pidThrottle.Out = 0;
        m_u0 = 0;
    }
}
void Longitudinal_Controller::Longitudinal_CAS() {
    int mode = 0;
    brake_f=0;
    if (u_desire < 0){
        states = LOW;
    }
    else if (kmph<1 && u_desire>1){
        states = STARTUP;
    }
    else{
        if (u_desire - kmph > 5.4){
            states = ACC;
        }
        else if (u_desire - kmph > -4){
            states = ADJUST;
        }
            /*else if (u_desire - kmph > 0)
            {
                states = STABLE;
            }*/
        else{
            states = BRAKE;
        }
    }
    std::cout <<"states:"<< states <<std::endl;
    switch (states)
    {
        case STARTUP://期望速度大而实际速度小
            throttle = 1;
            brake = 0;
            mode = 2;
            brake = 0;
            throttle = 10;
            break;
        case LOW://期望速度小
            if (u_desire - kmph < 0)
            {//制动
                throttle = 0;
                brake = 1;
                mode = 2;
                brake = 0.7 * (kmph - u_desire);
                if (brake < 10){
                    brake = 10;
                }
                //brake = 10;
                throttle = 0;
            }
            else{
                throttle = 1;
                brake = 0;
                if (u_desire - kmph > 1){
                    mode = 2;
                    brake = 0;
                    throttle = 12;
                }
                else{
                    mode = 2;
                    brake = 0;
                    throttle = 0;
                }
            }
            break;
        case ACC://期望速度远大于实际速度
            throttle = 1;
            brake = 0;
            mode = 2;
            brake = 0;
            throttle = 15;//加速时无法提速，可提高此参数
            break;
        case ADJUST://期望速度略大于实际速度
            if (u_desire - kmph > -1){
                throttle = 1;
                brake = 0;
                mode = 2;
                brake = 0;
                double a4;
                m_pidLongitudinal.Kp = 0.07;//调节阶段，PID
                m_pidLongitudinal.Ki = 0.00;
                m_pidLongitudinal.Kd = 1;
                m_pidLongitudinal.Kc = 0.03;
                m_pidLongitudinal.OutMax = 1;//1 + 0.01 * u_desire;//输出最大值 //
                m_pidLongitudinal.OutMin = 0;//输出最小值
                m_pidLongitudinal.Ref = u_desire;//过程值
                m_pidLongitudinal.Fdb = kmph;//反馈值

                pid_reg3_calc(&m_pidLongitudinal);
                a4 = m_pidLongitudinal.Out;//调节阶段，PID输出偏移
                //brake = 0;
                throttle = a4 * 100;
            }
            else{
                throttle = 0;
                brake = 1;
                mode = 2;
                throttle = 0;
                brake = 1 * (kmph - u_desire);
            }

            //throttle = 25;
            break;
            //case STABLE://期望速度约等于实际速度
            //	mode = 1;
            //	brake = 0;
            //	throttle = 0;
            //	break;
        case BRAKE://期望速度小于实际速度
            throttle = 0;
            brake = 1;
            mode = 2;
            if (kmph<10){
                brake_f=10;
                brake=brake+(brake_f-brake)*0.1;
            }
            else if (kmph<20){
                brake_f=15;
                brake=brake+(brake_f-brake)*0.1;
            }
            else if(kmph<30){
                brake_f=20;
                brake=brake+(brake_f-brake)*0.1;
            }
            else{
                brake_f=20;
                brake=brake+(brake_f-brake)*0.1;
            }
            throttle = 0;
            break;
        default:
            break;
    }

//    VehicleControl.PC_BREAK_VAULE=brake;//JAC制动输出值 加速度
//    VehicleControl.PC_Ped_VAULE=throttle;//JAC油门输出值 加速度

    //return mode;
}
void Longitudinal_Controller::Callback_DesireVelocity(const Traj_tracking::u_desire::ConstPtr& msg){
    u_desire = msg->u_desire;
    std::cout<<"Get Current velocity!"<<std::endl;
}
double Longitudinal_Controller::desiredSpeedFilter()
{
    double u = 0;
    const double T = 1;
    const double k = 0.06;
    if (u_desire > m_u0) {
        //u = (u_desire + 10 * T * m_u0) / (1 + 10 * T);
        u = u_desire * k + m_u0 * (1 - k) + 5;
        if (u >= u_desire)
            u = u_desire;
    }
    else
        u = u_desire;
    m_u0 = u;
    //filter << u_desire << ' ' << u << ' ' << m_u0 << endl;
    return u;
}
double Longitudinal_Controller::desiredSpeedFilter2()
{
    double u0 = 0;
    double m_u = Curr_LongState.velocity * 3.6;
    const double k0 = 0.05;
    if (m_u > u_desire){
        u0 = m_u * (1 - k0) - u_desire * k0 - 5;
        if (u0 <= u_desire)
            u0 = u_desire;
    }
    else
        u0 = u_desire;

    return u0;
}