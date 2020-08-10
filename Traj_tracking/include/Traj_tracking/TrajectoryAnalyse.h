#pragma once
#include "PathTracking.h"
#include <vector>

struct input_trajType{
	int PointID;
	double lng;
	double lat;
	double head;
	double cur;
};

struct PointType {
	double lng;
	double lat;
	double head;
	double velocity;
	double yawrate;
};

class Trajectory_Analyse {
	//friend class Lateral_Controller;
public:
	std::vector<input_trajType>reference_trajectory;

	void LoadReferenceTrajectory();
	void ComputeTargetCurvature();
	input_trajType FindTargetPoint(PointType);
	double ComputeGPSDistance(double,double,double,double);
	double AngleConvertRadin(double);
};
