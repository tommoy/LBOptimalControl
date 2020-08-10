#include "ros/ros.h"
#include <fstream>
#include <chrono>
#include "Traj_tracking/parameter.h"
#include "Traj_tracking/pid_reg3.h"
#include "Traj_tracking/PathTracking.h"
#include "Traj_tracking/TrajectoryAnalyse.h"
#include "Traj_tracking/MotionControl.h"
#include "Traj_tracking/GP_Model.h"

int main(int argc,char **argv){
    ros::init(argc,argv,"Control_Node");

    Lateral_Controller LateralControl;
    Longitudinal_Controller LongControl;
    Trajectory_Analyse TrajectoryAnalyse;
    Residual_Part ResiudualPart;

//    LongControl.u_desire = 5;

    LongControl.InitParam();
    LateralControl.InitParam();

    LateralControl.InitMatrix();
//    LateralControl.MPC_InitMatrix();

    TrajectoryAnalyse.LoadReferenceTrajectory();
    TrajectoryAnalyse.ComputeTargetCurvature();

    auto gpi = ResiudualPart.Optimize();

    std::ofstream data_RealTraj("data_RealTraj.txt");
    data_RealTraj << "Longitude" << ' ' << "Latitude" << ' ' << "Heading" << ' ' << "YawRate" << ' ' << "Velocity_desire" << ' '
    << "Velocity" << " " <<  "Throttle" << " " <<  "Brake"<< " " <<  "Steer_angle" << " " <<  "Feedback" << " " <<  "Feedforward"
            << " " <<  "TargetPoint" << " " <<  "Lateral_error" << " " <<  "Heading_error" << std::endl;


    std::ofstream TimeCost_AnalyticSolution("TimeCost.txt");
    TimeCost_AnalyticSolution << "LQR_Horizen" << " " << "LQR_timeCost" <<" "<<"Prediction_timeCost"<< std::endl;
//    TimeCost_AnalyticSolution << "LQR_Horizen" << " " << "LQR_timeCost" << std::endl;

//    std::ofstream TimeCost_mpc("TimeCost_mpc.txt");
////    TimeCost_mpc << "MPC_Horizen" << " " << "MPC_timeCost" <<" "<<"Prediction_timeCost"<< std::endl;
//    TimeCost_mpc << "MPC_Horizen" << " " << "MPC_timeCost" << std::endl;

    ros::NodeHandle n;
    ros::Publisher pub_command = n.advertise<Traj_tracking::MotionControl>("MotionControl", 1000);
    ros::Subscriber gps_sub = n.subscribe<Traj_tracking::ivsensorgps>("/gpsimu",1000, &Lateral_Controller::Callback_gps, &LateralControl);
    ros::Subscriber vel_sub = n.subscribe<Traj_tracking::u_desire>("setspeed",1000, &Longitudinal_Controller::Callback_DesireVelocity, &LongControl);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        LateralControl.Curr_State.lng = 117.1218421;
        LateralControl.Curr_State.lat = 31.8475827;
        LateralControl.Curr_State.head = 39.1380004883;
        LateralControl.Curr_State.velocity = 10;
        LateralControl.Curr_State.yawrate = 0;

        LongControl.kmph = LateralControl.Curr_State.velocity * 3.6;

        input_trajType TargetPoint = TrajectoryAnalyse.FindTargetPoint(LateralControl.Curr_State);

        std::cout<<"TargetPoint.PointID:  "<< TargetPoint.PointID <<std::endl;

        if(LateralControl.Curr_State.velocity == 0){
            LateralControl.Curr_State.velocity = 0.1;
        }

        auto start_1 = std::chrono::system_clock::now();
        LateralControl.Upadte_Discrete_A(LateralControl.Curr_State.velocity);
        LateralControl.ComputeStateMatrix(TargetPoint, LongControl.u_desire);
//        std::cout<<"Curr_State.head:  "<<LateralControl.Curr_State.head<<std::endl;
//        std::cout<<"TargetPoint.head:  "<<TargetPoint.head<<std::endl;
        LateralControl.SloveDiscreteLQR();
        LateralControl.ComputeFeedforward(TargetPoint);
        LateralControl.Constraint_Transform();
        auto end_LQR = std::chrono::system_clock::now();
        auto duration_LQR = std::chrono::duration_cast<std::chrono::microseconds>(end_LQR - start_1);
        auto lqr_time = double(duration_LQR.count())* std::chrono::milliseconds::period::num / std::chrono::milliseconds::period::den;
        std::cout<<"LQR spent: "<< lqr_time << " ms." << std::endl;

        ResiudualPart.Predict(gpi,30,20);
        auto end_Optimize = std::chrono::system_clock::now();
        auto duration_Prediction = std::chrono::duration_cast<std::chrono::microseconds>(end_Optimize - end_LQR);
        auto Prediction_time = double(duration_Prediction.count())* std::chrono::milliseconds::period::num / std::chrono::milliseconds::period::den;
        std::cout<<"Prediction spent: "<< Prediction_time << " ms." << std::endl;
        TimeCost_AnalyticSolution << LQR_iterative <<" "<< lqr_time << " " << Prediction_time <<std::endl;
//        TimeCost_AnalyticSolution << LQR_iterative <<" "<< lqr_time  <<std::endl;



//        auto start_mpc = std::chrono::system_clock::now();
//        LateralControl.Curr_State.velocity = 20;
//        LateralControl.MPC_Update_Discrete_AC(LateralControl.Curr_State.velocity);
//        LateralControl.MPC_Update_Matrix();
//        LateralControl.MPC_QPSolver();
//
//        auto end_mpc = std::chrono::system_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_mpc-start_mpc);
//        auto mpc_time = double(duration.count())* std::chrono::milliseconds::period::num / std::chrono::milliseconds::period::den;
//        std::cout<<"mpc spent: "<< mpc_time << " ms." << std::endl;
//
//        ResiudualPart.Predict(gpi,30,20);
//        auto end_Optimize = std::chrono::system_clock::now();
//        auto duration_Prediction = std::chrono::duration_cast<std::chrono::microseconds>(end_Optimize - end_mpc);
//        auto Prediction_time = double(duration_Prediction.count()) * std::chrono::milliseconds::period::num / std::chrono::milliseconds::period::den;
//        std::cout << "Prediction spent: " << Prediction_time << " ms." << std::endl;
//        TimeCost_mpc << MPC_Horizen <<" "<< mpc_time << " " << Prediction_time <<std::endl;
//        TimeCost_mpc << MPC_Horizen <<" "<< mpc_time <<std::endl;

        LongControl.Longitudinal_CAS();

//        std::cout<<"**Matrix_State:"<<std::endl;
//        std::cout<<LateralControl.lat_matrix.matrix_State<<std::endl;
//        std::cout<<std::endl;

//        std::cout<<"**Matrix_Ad:"<<std::endl;
//        std::cout<<LateralControl.lat_matrix.matrix_A<<std::endl;
//        std::cout<<std::endl;


//        std::cout<<"**Command:"<<std::endl;
//        std::cout<<"K: "<<LateralControl.lat_matrix.matrix_K<<std::endl;
//        std::cout<<"feedback: "<<LateralControl.tire_angle_feedback * 180/pi_1<<std::endl;
//        std::cout<<"feedforward: "<<LateralControl.tire_angle_feedforward * 180/pi_1<<std::endl;
//        std::cout<<"Steer_Angle(deg): "<<LateralControl.steer_angle<<std::endl;
//        std::cout<<"throttle: "<<LongControl.throttle<<std::endl;

        std::vector<double>::iterator it;
        for (it = LateralControl.lateral_error_set.begin(); it != LateralControl.lateral_error_set.end(); it++) {
            std::cout << *it <<" ";
        }
        std::cout<<std::endl;
        std::cout<<"*************************************************************************************"<<std::endl<<std::endl;

        Traj_tracking::MotionControl Control_cmd;

        Control_cmd.PC_BREAK_VAULE = LongControl.brake;
        Control_cmd.PC_Ped_VAULE = 5;
//        Control_cmd.PC_Ped_VAULE = LongControl.throttle;
        Control_cmd.PC_STR_ANGLE = LateralControl.steer_angle;

        data_RealTraj << std::setprecision(12) <<LateralControl.Curr_State.lng<<" "<<LateralControl.Curr_State.lat
                      << " " << LateralControl.Curr_State.head * 180/pi_1 << " " << LateralControl.Curr_State.yawrate << " " << LongControl.u_desire << " " <<
                      LongControl.kmph << " " << Control_cmd.PC_Ped_VAULE << " " << Control_cmd.PC_BREAK_VAULE << " " << Control_cmd.PC_STR_ANGLE << " " <<
                      LateralControl.tire_angle_feedback << " " << LateralControl.tire_angle_feedforward << " " << TargetPoint.PointID << " " <<
                      LateralControl.lat_matrix.matrix_State(0) << " " << LateralControl.lat_matrix.matrix_State(2) << " " <<std::endl;

        pub_command.publish(Control_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

