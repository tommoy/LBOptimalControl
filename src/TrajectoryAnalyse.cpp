#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <vector>
#include "Traj_tracking/parameter.h"
#include "Traj_tracking/PathTracking.h"
#include "Traj_tracking/TrajectoryAnalyse.h"

void Trajectory_Analyse::LoadReferenceTrajectory() {

	//std::ifstream reference_path("trajectory_example.txt");
    std::ifstream reference_path("/home/jcm/catkin_LBOC/roadpoint_line.txt");

	//while (!reference_path.eof()) {
	//	CvPoint3D64f seqPoint_temp;
	//	reference_path >> seqPoint_temp.x >> seqPoint_temp.y >> seqPoint_temp.z;
	//	cvSeqPush(reference_trajectory, &seqPoint_temp);
	//}
	//for (int j = 0; j < reference_trajectory->total; j++) {
	//	CvPoint3D64f point = *(CvPoint3D64f*)cvGetSeqElem(reference_trajectory, j);
	//	std::cout << std::setprecision(12) << point.x << point.y << point.z << std::endl;
	//}

	while (!reference_path.eof()) {
		input_trajType point_temp;
		point_temp.cur = 0;
		reference_path >> point_temp.lng >> point_temp.lat >> point_temp.head;
		reference_trajectory.push_back(point_temp);
	}
	for (int i = 0; i < reference_trajectory.size(); i++) {
		reference_trajectory.at(i).PointID = i;
	}

//	std::vector<input_trajType>::iterator it;
//	for (it = reference_trajectory.begin(); it != reference_trajectory.end(); it++) {
//		std::cout << std::setprecision(12) << it->PointID << " "<< it->lng << " " << it->lat << " " << it->head << " " << it->cur << std::endl;
//	}
	std::cout << "Load Reference Road Successed!" << std::endl;
}

void Trajectory_Analyse::ComputeTargetCurvature() {
	std::vector<double>PointDis1;//Distance of consecutive points
	for (size_t i = 0; i < reference_trajectory.size() - 1; i++) {
		double temp_distance;
		temp_distance = ComputeGPSDistance(reference_trajectory.at(i).lng, reference_trajectory.at(i).lat,
			reference_trajectory.at(i + 1).lng, reference_trajectory.at(i + 1).lat);
		PointDis1.push_back(temp_distance);
//		std::cout<<temp_distance<<std::endl;
	}

	std::vector<double>PointDis2;//Distance of one apart points
	for (size_t i = 0; i < reference_trajectory.size() - 2; i++) {
		double temp_distance;
		temp_distance = ComputeGPSDistance(reference_trajectory.at(i).lng, reference_trajectory.at(i).lat,
			reference_trajectory.at(i + 2).lng, reference_trajectory.at(i + 2).lat);
		PointDis2.push_back(temp_distance);
//        std::cout<<temp_distance<<std::endl;
	}

    for (size_t i = 0; i < reference_trajectory.size() - 2; i++) {
        double angle_judge = PointDis1.at(i) + PointDis1.at(i + 1) - PointDis2.at(i);
        if(angle_judge >= 0) {
            double cosA = (pow(PointDis1.at(i), 2) + pow(PointDis1.at(i + 1), 2) - pow(PointDis2.at(i), 2)) /
                    (2 * PointDis1.at(i) * PointDis1.at(i + 1));
            double cur_temp = 2 * sqrt(1 - pow(cosA, 2)) / PointDis1.at(i + 1);
            reference_trajectory.at(i + 1).cur = cur_temp;
        }else {
            reference_trajectory.at(i + 1).cur = reference_trajectory.at(i).cur;
        }
    }
    reference_trajectory.at(0).cur = reference_trajectory.at(1).cur;
    reference_trajectory.at(reference_trajectory.size()-1).cur = reference_trajectory.at(reference_trajectory.size() - 2).cur;

    std::vector<input_trajType>::iterator it;
    for (it = reference_trajectory.begin(); it != reference_trajectory.end(); it++) {
        std::cout << it->cur << std::endl;
    }

	std::cout << "Get TargetCurvature" << std::endl;
}

input_trajType Trajectory_Analyse::FindTargetPoint(PointType curr_point) {
	std::vector<double>Dis_CurrToTraj;
	std::vector<input_trajType>::iterator it;
	for (it = reference_trajectory.begin(); it != reference_trajectory.end(); it++) {
		double Dis_temp = ComputeGPSDistance(curr_point.lng, curr_point.lat, it->lng, it->lat);
		Dis_CurrToTraj.push_back(Dis_temp);
	}
	//auto dis_min = std::min_element(Dis_CurrToTraj.begin(), Dis_CurrToTraj.end());
	//int PointID= dis_min - Dis_CurrToTraj.begin();
    int PointID = std::min_element(Dis_CurrToTraj.begin(), Dis_CurrToTraj.end()) - Dis_CurrToTraj.begin();

	std::vector<input_trajType>::iterator iter = reference_trajectory.begin();
	iter += PointID;
	//std::cout << std::setprecision(12) << iter->lng << std::endl;
	return *iter;

	std::cout<<"Get Target Point"<<std::endl;
}

double Trajectory_Analyse::ComputeGPSDistance(double lng1, double lat1, double lng2, double lat2) {
	if (lat1 == lat2 && lng1 == lng2)
		return 0;
	double radLat1 = AngleConvertRadin(lat1);
	double radLat2 = AngleConvertRadin(lat2);
	double a = radLat1 - radLat2;
	double b = AngleConvertRadin(lng1) - AngleConvertRadin(lng2);
	double s = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2)));
	s = s * EARTH_RADIUS * 1000;
	return s;
}
double Trajectory_Analyse::AngleConvertRadin(double Angle_value) {
	return Angle_value * pi_1 / 180.0;
}
