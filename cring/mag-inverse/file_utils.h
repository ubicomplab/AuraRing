#pragma once
#include "settings.h"
#include <vector>
#include "hand_models.h"
#include "mag_models.h"

struct DataFrame {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	array<double, NUM_RX*3> sensors;
	Eigen::Vector3d ring_pos;
	Eigen::Quaterniond ring_q;
	array<double, 4> joint_angles;
#ifdef USE_PALM
	Eigen::Vector3<double> palm_pos;
	Eigen::Quaterniond palm_q;
#endif
//#ifdef USE_MARKERS
//	vector<float> markers;
//#endif
};

struct per_frame_opt_parameters {
#if USE_JOINT
	array<double, 4> joint_angles;
	Eigen::Vector3d slack_pos;
	Eigen::Quaterniond slack_rot;
	HandModel hand_model;
#else
	Eigen::Vector3d ring_pos;
	Eigen::Quaterniond ring_rot;
#endif
};


void writePerFrameResults(string fileName, string trial, const Eigen::Vector3d& ring_pos, const Eigen::Quaterniond& ring_rot, const Eigen::Vector3d& ring_pos_actual, const Eigen::Quaterniond& ring_rot_actual, const per_frame_opt_parameters& hand_model,
	bool newFile);

void writeCalibration(string fileName, string trial, const CoilCalibrationModel<double>& coil, bool newFile);

vector<DataFrame> parseCSVFile(string fileName, string trial, char delimiter, unsigned int limit=0);
vector<DataFrame> parseProjectedCSVFile(string fileName, string trial, char delimiter, unsigned int limit = 0);

CalibrationModel<double> parseCalibratedFile(string fileName, string trial, char delimiter);

void writeHandModel(string fileName, string trial, const HandModel& hand_model);
void writeCSVFile(string fileName, string trial, vector<array<double, 4>> angles, vector<hand_model_data<double>> joints, vector<DataFrame> raw_data);

HandModel parseHandModel(string fileName, string trial);
vector<array<double, 4>> parseMatlab(string fileName, string trial);