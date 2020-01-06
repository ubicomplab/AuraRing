// mag-inverse.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "utils.h"
#include "settings.h"
#include "joint_ik.h"
#include <math.h>
#include <vector>
#include <algorithm>  // std::copy
#include <array>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/internal/port.h"
#include "ceres/jet.h"
#include "ceres/rotation.h"
#include "glog/logging.h"
#include "hand_models.h"
#include "mag_models.h"
#include "file_utils.h"
#include "comms.h"
#include <chrono>  // for high_resolution_clock
#include <gflags/gflags.h>


#define SOLVE_JOINT 1

constexpr unsigned int N = 0;
DEFINE_string(hand_model, "", "hand model csv file to load");
DEFINE_string(calibration, "", "calibration csv file to load");


bool newFile = true;

using namespace ceres;
using namespace Eigen;
using namespace std;




template <typename T>
void ComputeCost(const array<T, NUM_RX * 3>& sensor, const array<T, NUM_RX * 3>& sensor_pred, T* residual, const Vector3<T>& field1, const Vector3<T>& field2, const Vector3<T>& field3)
{
	Eigen::Map<const Eigen::Vector<T, NUM_RX * 3>> sensor_v(sensor.data());
	Eigen::Map<const Eigen::Vector<T, NUM_RX * 3>> sensor_pred_v(sensor_pred.data());

	//cout << sensor_v << sensor_pred_v << endl;

	Eigen::Vector<T, NUM_RX * 3> diff = (sensor_v - sensor_pred_v);
	for (unsigned int i = 0; i < NUM_RX * 3; i++)
	{
		Vector3<T> field;
		switch (i / 3) {
		case 0:
			field = field1;
			break;
		case 1:
			field = field2;
			break;
		case 2:
			field = field3;
			break;
		}
		if (field[i % 3] > 0.0025 || field[i % 3] < -0.0025)
		{
			//residual[i] = ceres::abs(diff[i]) * T(500000) *  ceres::abs(field[i % 3]);
			residual[i] = diff[i] * T(5000000);
			//residual[i] = diff[i] * T(500000) * field[i % 3];
		}
		else
		{
			residual[i] = T(0);
		}
		//print_jacobian(residual[i]);
		//printf("%f\n", D(residual[i]));
	}


}

double get_cost(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot, const array<double, NUM_RX * 3>& sensors, const CalibrationModel<double>& model) {
	Vector3d field;
	Eigen::Vector3d field_pred1 = forward_model(pos, rot, model.coil1, false, field);
	Eigen::Vector3d field_pred2 = forward_model(pos, rot, model.coil2, false, field);
	Eigen::Vector3d field_pred3 = forward_model(pos, rot, model.coil3, false, field);

	double error = 0;
	for (int j = 0; j < 3; j++) {
		error += pow(field_pred1[j] - sensors[j], 2) * 1000;
		error += pow(field_pred2[j] - sensors[j + 3], 2) * 1000;
		error += pow(field_pred3[j] - sensors[j + 6], 2) * 1000;
	}
	return error;
}

class PerFrameCostFunctor
{
public:
	void* operator new(size_t i)
	{
		return _mm_malloc(i, 16);
	}

	void operator delete(void* p)
	{
		_mm_free(p);
	}
	PerFrameCostFunctor(const CalibrationModel<double>& model) : model_(model) {}

	template <typename T> bool operator()(const T* const p_pos, const T* const p_rot, T* residual) const {
		Eigen::Map<const Eigen::Vector3<T>> pos(p_pos);
		Eigen::Map<const Eigen::Quaternion<T>> rot(p_rot);


		CalibrationModel<T> model_T = model_.cast<T>();
		Vector3<T> field1, field2, field3;

		Eigen::Vector3<T> sensor_1_pred = forward_model<T>(pos, rot, model_T.coil1, false, field1);
		Eigen::Vector3<T> sensor_2_pred = forward_model<T>(pos, rot, model_T.coil2, false, field2);
		Eigen::Vector3<T> sensor_3_pred = forward_model<T>(pos, rot, model_T.coil3, false, field3);

		array<T, NUM_RX * 3> sensor;
		array<T, NUM_RX * 3> sensor_pred;
		for (unsigned int i = 0; i < NUM_RX * 3; i++) {
			sensor[i] = T(signed_sensors[i]);
		}
		for (unsigned int i = 0; i < 3; i++) {
			sensor_pred[i] = sensor_1_pred[i];
			sensor_pred[i + 3] = sensor_2_pred[i];
			sensor_pred[i + 6] = sensor_3_pred[i];
		}

		ComputeCost(sensor, sensor_pred, residual, field1, field2, field3);


		return true;
	}
	void UpdateData(const sensor_data& sensors) {
		signed_sensors = sensors;
	}
private:
	CalibrationModel<double> model_;
	sensor_data signed_sensors;
};


class SmoothPositionCostFunctor
{
public:
	void* operator new(size_t i)
	{
		return _mm_malloc(i, 16);
	}

	void operator delete(void* p)
	{
		_mm_free(p);
	}
	SmoothPositionCostFunctor(const Eigen::Vector3d& ring_pos, const Eigen::Quaterniond& ring_rot) :
		ring_pos_(ring_pos), ring_rot_(ring_rot) {}

	template <typename T> bool operator()(const T* const ring_pos, const T* const ring_rot, T* residual) const {
		Eigen::Map<const Eigen::Vector3<T>> pos(ring_pos);
		Eigen::Map<const Eigen::Quaternion<T>> rot(ring_rot);

		Eigen::Map<const Eigen::Vector<double, 3>> last_position(ring_pos_.data());

		T position_error = (pos - last_position.cast<T>()).squaredNorm();
		Eigen::Quaternion<T> rot_diff = rot.conjugate() * ring_rot_.cast<T>();
		Eigen::AngleAxis<T> rot_diff_AA(rot_diff);
		T rot_error = rot_diff_AA.angle();
		//residual[0] = (position_error + rot_error)*T(1);
		residual[0] = (position_error)* T(1);
		return true;
	}
	void UpdateData(const Eigen::Vector3d& ring_pos, const Eigen::Quaterniond& ring_rot) {
		ring_pos_ = ring_pos;
		ring_rot_ = ring_rot;
	}
private:
	Eigen::Vector3d ring_pos_;
	Eigen::Quaterniond ring_rot_;
};


class SmoothRotationCostFunctor
{
public:
	void* operator new(size_t i)
	{
		return _mm_malloc(i, 16);
	}

	void operator delete(void* p)
	{
		_mm_free(p);
	}
	SmoothRotationCostFunctor(const Eigen::Quaterniond& ring_rot) : ring_rot_(ring_rot) {}

	template <typename T> bool operator()(const T* const ring_rot, T* residual) const {
		Eigen::Map<const Eigen::Quaternion<T>> rot(ring_rot);

		Eigen::Quaternion<T> rot_diff = rot.conjugate() * ring_rot_.cast<T>();
		Eigen::AngleAxis<T> rot_diff_AA(rot_diff);
		T rot_error = rot_diff_AA.angle();

		Vector3<T> x_proj = rot * Vector3<T>(T(1), T(0), T(0));

		residual[0] = x_proj[0];// rot_error;
		return true;
	}
	void UpdateData(const Eigen::Quaterniond& ring_rot) {
		ring_rot_ = ring_rot;
	}
private:
	Eigen::Quaterniond ring_rot_;
};

struct ProblemInfo
{
	PerFrameCostFunctor* cost;
	ceres::Solver::Options options;
	ceres::Problem frame_problem;
	ceres::Solver::Summary summary;
	per_frame_opt_parameters param;
};

ProblemInfo& init_problem(const CalibrationModel<double>& calibration_model)
{
	std::cout << "Start Per Frame Opt" << endl;

	static ProblemInfo problem;

	
	per_frame_opt_parameters param_lower;
	per_frame_opt_parameters param_upper;

	problem.param.ring_pos = Vector3d(-15, 0, -120);
	problem.param.ring_rot = Quaterniond(1, 0, 0, 0);


	param_lower.ring_pos = { -60, -130, -150 };
	param_lower.ring_rot = Eigen::Quaterniond(-1, -1, -1, -1);

	param_upper.ring_pos = { 85, 90, -50 };
	param_upper.ring_rot = Eigen::Quaterniond(1, 1, 1, 1);


	problem.frame_problem.AddParameterBlock(problem.param.ring_pos.data(), 3);
	problem.frame_problem.AddParameterBlock(problem.param.ring_rot.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());

	// set bounds on parameters
	for (unsigned int i = 0; i < 3; ++i) {
		problem.frame_problem.SetParameterLowerBound(problem.param.ring_pos.data(), i, param_lower.ring_pos[i]);
		problem.frame_problem.SetParameterUpperBound(problem.param.ring_pos.data(), i, param_upper.ring_pos[i]);
	}
	for (unsigned int i = 0; i < 4; ++i) {
		problem.frame_problem.SetParameterLowerBound(problem.param.ring_rot.coeffs().data(), i, param_lower.ring_rot.coeffs()[i]);
		problem.frame_problem.SetParameterUpperBound(problem.param.ring_rot.coeffs().data(), i, param_upper.ring_rot.coeffs()[i]);
	}


	HandModel null_model;
	problem.cost = new PerFrameCostFunctor(calibration_model);
	ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PerFrameCostFunctor, NUM_RX * 3, 3, 4>(problem.cost);
	problem.frame_problem.AddResidualBlock(cost_function, NULL, problem.param.ring_pos.data(), problem.param.ring_rot.coeffs().data());


	problem.summary = Solver::Summary();
	problem.options = Solver::Options();
	problem.options.minimizer_progress_to_stdout = false;
	problem.options.num_threads = 1;
	problem.options.max_num_iterations = 100;

	return problem;
}

void solve_frame(ProblemInfo& problem, const sensor_data& sensors)
{
	auto start_time = std::chrono::high_resolution_clock::now();
	// Run the solver!
	problem.cost->UpdateData(sensors);

	ceres::Solve(problem.options, &problem.frame_problem, &problem.summary);

	auto now = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = now - start_time;
	start_time = now;
	//printf("%f sec\n", elapsed.count());

	return;
}


int main(int argc, char* argv[])
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	string MODE = "";
	std::cout << "RUNNING PER FRAME OPT" << endl;
	CalibrationModel<double> calibration_model = {};
	std::cout << "STARTED PARSING DATA" << endl;

#if SOLVE_JOINT
	HandModel hand_model = parseHandModel(CERES_DIRECTORY + "handModel", FLAGS_hand_model);
	hand_model.bone_lengths[0] = 95;
	hand_model.bone_lengths[1] = 33;
	hand_model.wrist_offset = Vector3d(0, -30, 10);
	
	JointProblemInfo& joint_problem = init_per_frame_problem(hand_model, false);
#endif

	string calib = FLAGS_calibration;
	calibration_model = parseCalibratedFile(CERES_DIRECTORY + "calibrate", calib, ' ');
	sensor_data data;
	int ret_val = 0;

	ProblemInfo& problem = init_problem(calibration_model);
	
	init_winsock();
	SOCKET s;
	init_socket(s);
	SOCKET s2;
	init_out_socket(s2);
	double buffer[25];
	hand_model_data<double> solved_hand;
	int i = 0;
	while (1)
	{
		ret_val = get_sensor_data_from_socket(s, data);
		sensor_data temp = data;
		data[0] = temp[0];
		data[1] = temp[4];
		data[2] = temp[2];
		data[3] = temp[1];
		data[4] = temp[5];
		data[5] = temp[3];
		data[6] = temp[6];
		data[7] = temp[8];
		data[8] = temp[7];

		if (ret_val == 0) {
			solve_frame(problem, data);
			//problem.param.ring_rot = Quaterniond(1, 0, 0, 0);
#if SOLVE_JOINT
			DataFrame frame;
			frame.ring_pos = problem.param.ring_pos;
			frame.ring_q = problem.param.ring_rot;
			solve_joint_frame(joint_problem, frame);
			solved_hand = compute_hand_model<double>(hand_model, joint_problem.joint_angles[0], joint_problem.joint_angles[1], joint_problem.joint_angles[2], joint_problem.joint_angles[3], Vector3d(0, 0, 0), Quaterniond(1, 0, 0, 0));
#endif
			i = 0;
			memcpy_s(buffer + i, sizeof(buffer), problem.param.ring_pos.data(), sizeof(double) * 3);
			i += 3;
			memcpy_s(buffer + i, sizeof(buffer) - i * sizeof(double), problem.param.ring_rot.coeffs().data(), sizeof(double) * 4);
			i += 4;
#if SOLVE_JOINT
			memcpy_s(buffer + i, sizeof(buffer) - i * sizeof(double), joint_problem.joint_angles.data(), sizeof(double) * 4);
			i += 4;
			memcpy_s(buffer + i, sizeof(buffer) - i * sizeof(double), solved_hand.ring_pos.data(), sizeof(double) * 3);
			i += 3;
			memcpy_s(buffer + i, sizeof(buffer) - i * sizeof(double), solved_hand.ring_rot.coeffs().data(), sizeof(double) * 4);
			i += 4;
			memcpy_s(buffer + i, sizeof(buffer) - i * sizeof(double), solved_hand.knuckle_pos.data(), sizeof(double) * 3);
			i += 3;
			memcpy_s(buffer + i, sizeof(buffer) - i * sizeof(double), solved_hand.knuckle_rot.coeffs().data(), sizeof(double) * 4);
			i += 4;
#endif
			
			send_solution_to_socket(s2, buffer, sizeof(buffer));
		}
	}
	std::cout << "Press any key to exit...";
	cin.get();
}

