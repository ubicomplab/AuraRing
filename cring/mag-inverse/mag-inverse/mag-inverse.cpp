// mag-inverse.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "utils.h"
#include "settings.h"
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
#include <chrono>  // for high_resolution_clock

#define HALLUCINATE_SIM_DATA 0
#define CHEAT 0
#define TAKE_CLOSEST 0
#define USE_DIPOLE 0
#define NOISE 0.00001
#define VERBOSE 0



DEFINE_string(trial, "", "trial to process");
DEFINE_string(calibration, "", "calibration file");
DEFINE_int32(limit, 0, "only process this many frames");

#if USE_JOINT
string HAND_MODEL = "";
#endif
bool newFile = true;

using namespace ceres;
using namespace Eigen;
using namespace std;

#if APPLY_SIGN_CORRECTION
#if USE_DIPOLE
array<int8_t, 6> forced_sign_bits = { 0, 1, 0, -1, 0, 0 };
#else
array<int8_t, 6> forced_sign_bits = { 0, 1, 1, 1, 1, 1 };
#endif
#endif



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
		if (field[i%3] > 0.0025 || field[i % 3] < -0.0025)
		{
			residual[i] = diff[i] * T(5000000);
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
#if NUM_RX == 3
		error += pow(field_pred3[j] - sensors[j + 6], 2) * 1000;
#endif
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
	PerFrameCostFunctor(const CalibrationModel<double>& model, const DataFrame& data, const HandModel& hand_model) : model_(model), data_(data), sign_(0), hand_model_(hand_model) {}

#if USE_JOINT
	template <typename T> bool operator()(const T* const joint_angles, const T* const slack_pos, const T* const slack_rot, const T* const wrist_offset, const T* const wrist_offset_rot,
		const T* const bone_lengths, const T* const cmc_alpha, T* residual) const {
		const T wrist_theta = joint_angles[0];
		const T wrist_phi = joint_angles[1];
		const T finger_theta = joint_angles[2];
		const T finger_phi = joint_angles[3];

		Eigen::Map<const Eigen::Vector3<T>> slack_pos_(slack_pos);
		Eigen::Map<const Eigen::Quaternion<T>> slack_rot_(slack_rot);
		Eigen::Map<const Eigen::Vector3<T>> wrist_offset_(wrist_offset);
		Eigen::Map<const Eigen::Quaternion<T>> wrist_offset_rot_(wrist_offset_rot);
		const array<T, 2> bone_lengths_ = { bone_lengths[0], bone_lengths[1] };
		const T cmc_alpha_ = *cmc_alpha;

		hand_model_data<T> solved_model = compute_hand_model<T>(wrist_offset_, wrist_offset_rot_, bone_lengths_, cmc_alpha_, wrist_theta, wrist_phi, finger_theta, finger_phi, 
			slack_pos_ + hand_model_.base_slack_pos.cast<T>(), slack_rot_ * hand_model_.base_slack_rot.cast<T>());
		const Eigen::Vector3<T> pos = solved_model.ring_pos;
		const Eigen::Quaternion<T> rot = solved_model.ring_rot;
#else
	template <typename T> bool operator()(const T* const p_pos, const T* const p_rot, T* residual) const {
		Eigen::Map<const Eigen::Vector3<T>> pos(p_pos);
		Eigen::Map<const Eigen::Quaternion<T>> rot(p_rot);
		//printf("Mag: %f\n", D(rot.norm()));
#endif


		CalibrationModel<T> model_T = model_.cast<T>();
		Vector3<T> field1, field2, field3;
#if HALLUCINATE_SIGN or APPLY_SIGN_CORRECTION
		Eigen::Vector3<T> sensor_1_pred = forward_model<T>(pos, rot, model_T.coil1, true, field1); // Preserve sign
		Eigen::Vector3<T> sensor_2_pred = forward_model<T>(pos, rot, model_T.coil2, true, field2); // Preserve sign
		Eigen::Vector3<T> sensor_3_pred = forward_model<T>(pos, rot, model_T.coil3, true, field3); // Preserve sign
#else
		Eigen::Vector3<T> sensor_1_pred = forward_model<T>(pos, rot, model_T.coil1, false, field1);
		Eigen::Vector3<T> sensor_2_pred = forward_model<T>(pos, rot, model_T.coil2, false, field2);
		Eigen::Vector3<T> sensor_3_pred = forward_model<T>(pos, rot, model_T.coil3, false, field3);
#endif
		array<T, NUM_RX * 3> sensor;
		array<T, NUM_RX * 3> sensor_pred;
		for (unsigned int i = 0; i < NUM_RX * 3; i++) {
			sensor[i] = T(signed_sensors[i]);
		}
		for (unsigned int i = 0; i < 3; i++) {
			sensor_pred[i] = sensor_1_pred[i];
			sensor_pred[i + 3] = sensor_2_pred[i];
#if NUM_RX == 3
			sensor_pred[i + 6] = sensor_3_pred[i];
#endif
		}

		ComputeCost(sensor, sensor_pred, residual, field1, field2, field3);


		return true;
	}
	void UpdateData(const DataFrame& data, uint8_t sign) {
		data_ = data;
		sign_ = sign;
		uint8_t sign_bit_idx = 0;
		int8_t sign_bit = 0;
#if APPLY_SIGN_CORRECTION
		//printf("Sensors: ");
		for (unsigned int i = 0; i < 6; i++)
		{
			if (forced_sign_bits[i] == 0)
			{
				sign_bit = ((sign_ >> sign_bit_idx) & 0x01) * 2 - 1;
				sign_bit_idx++;
			}
			else
			{
				sign_bit = forced_sign_bits[i];
			}

			signed_sensors[i] = data_.sensors[i] * sign_bit;
			//printf("%f, ", signed_sensors[i]);
		}
#elif HALLUCINATE_SIGN
		Vector3d field;
		Eigen::Vector3d sensor_1_pred = forward_model(data.ring_pos, data.ring_q, model_.coil1, true, field); // Preserve sign
		Eigen::Vector3d sensor_2_pred = forward_model(data.ring_pos, data.ring_q, model_.coil2, true, field); // Preserve sign
		Eigen::Vector3d sensor_3_pred = forward_model(data.ring_pos, data.ring_q, model_.coil3, true, field); // Preserve sign
		signed_sensors = data_.sensors;
		for (unsigned int i = 0; i < 3; i++)
		{
			signed_sensors[i] *= sgn(sensor_1_pred[i]);
			signed_sensors[i+3] *= sgn(sensor_2_pred[i]);
#if NUM_RX == 3
			signed_sensors[i + 6] *= sgn(sensor_3_pred[i]);
#endif
		}
#else
		signed_sensors = data_.sensors;
#endif
		//printf("\n");
	}
private:
	CalibrationModel<double> model_;
	DataFrame data_;
	array<double, NUM_RX*3> signed_sensors;
	uint8_t sign_;
	HandModel hand_model_;
};

class SlippageCostFunctor
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
	SlippageCostFunctor(HandModel model) : model_(model) {}

	template <typename T> bool operator()(const T* const wrist_offset, const T* const wrist_offset_rot, const T* const bone_lengths, T* residual) const {
		Eigen::Map<const Eigen::Vector<T, 3>> wrist_offset_(wrist_offset);
		Eigen::Map<const Eigen::Quaternion<T>> wrist_offset_rot_(wrist_offset_rot);
		Eigen::Map<const Eigen::Vector<T, 2>> bone_lengths_(bone_lengths);

		Eigen::Map<const Eigen::Vector<double, 2>> last_bone_lengths_(model_.bone_lengths.data());

		T wrist_error = (wrist_offset_ - model_.wrist_offset.cast<T>()).squaredNorm();
		Eigen::Quaternion<T> rot_diff = wrist_offset_rot_.conjugate() * model_.wrist_offset_rot.cast<T>();
		Eigen::AngleAxis<T> rot_diff_AA(rot_diff);
		T wrist_rot_error = rot_diff_AA.angle();
		T bone_error = (bone_lengths_ - last_bone_lengths_.cast<T>()).squaredNorm();
		residual[0] = (wrist_error + wrist_rot_error + bone_error)*T(0.1);
		return true;
	}
	void UpdateData(HandModel model) {
		model_ = model;
	}
private:
	HandModel model_;
};

class SmoothJointsCostFunctor
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
	SmoothJointsCostFunctor(const HandModel& hand_model) : hand_model_(hand_model){}

	template <typename T> bool operator()(const T* const joint_angles, const T* const slack_pos, const T* const slack_rot, const T* const wrist_offset, const T* const wrist_offset_rot,
		const T* const bone_lengths, const T* const cmc_alpha, T* residual) const {
		const T wrist_theta = joint_angles[0];
		const T wrist_phi = joint_angles[1];
		const T finger_theta = joint_angles[2];
		const T finger_phi = joint_angles[3];

		Eigen::Map<const Eigen::Vector3<T>> slack_pos_(slack_pos);
		Eigen::Map<const Eigen::Quaternion<T>> slack_rot_(slack_rot);
		Eigen::Map<const Eigen::Vector3<T>> wrist_offset_(wrist_offset);
		Eigen::Map<const Eigen::Quaternion<T>> wrist_offset_rot_(wrist_offset_rot);
		const array<T, 2> bone_lengths_ = { bone_lengths[0], bone_lengths[1] };
		const T cmc_alpha_ = *cmc_alpha;

		hand_model_data<T> solved_model = compute_hand_model<T>(wrist_offset_, wrist_offset_rot_, bone_lengths_, cmc_alpha_, wrist_theta, wrist_phi, finger_theta, finger_phi,
			slack_pos_ + hand_model_.base_slack_pos.cast<T>(), slack_rot_ * hand_model_.base_slack_rot.cast<T>());
		const Eigen::Vector3<T> pos = solved_model.ring_pos;
		const Eigen::Quaternion<T> rot = solved_model.ring_rot;
		const Eigen::Vector3<T> last_pos_T = last_pos_.cast<T>();
		const Eigen::Quaternion<T> last_rot_T = last_rot_.cast<T>();

		residual[0] = (pos[0] - last_pos_T[0]) * T(.1);
		residual[1] = (pos[1] - last_pos_T[1]) * T(.1);
		residual[2] = (pos[2] - last_pos_T[2]) * T(.1);
		AngleAxis<T> rot_error = AngleAxis<T>(rot * last_rot_T.conjugate());
		residual[3] = T(0);// rot_error.angle();
		return true;
	}
	void UpdateData(const Vector3d& last_pos, const Quaterniond& last_rot) {
		last_pos_ = last_pos;
		last_rot_ = last_rot;
	}
private:
	Vector3d last_pos_;
	Quaterniond last_rot_;
	HandModel hand_model_;
};


class NeutralJointsCostFunctor
{
public:

	NeutralJointsCostFunctor()  {}

	template <typename T> bool operator()(const T* const joint_angles, T* residual) const {
		const T wrist_theta = joint_angles[0];
		const T wrist_phi = joint_angles[1];
		const T finger_theta = joint_angles[2];
		const T finger_phi = joint_angles[3];

		residual[0] = (wrist_theta - T(-7 * PI / 180)) * T(1);
		residual[1] = (wrist_phi - T(-5 * PI / 180)) * T(1);
		residual[2] = (finger_theta - T(-17 * PI / 180)) * T(1);
		residual[3] = (finger_phi - T( 5* PI / 180)) * T(1);
		return true;
	}

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
	SmoothPositionCostFunctor(const Eigen::Vector3d ring_pos, const Eigen::Quaterniond ring_rot) : 
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
		residual[0] = (position_error) * T(1); 
		return true;
	}
	void UpdateData(const Eigen::Vector3d ring_pos, const Eigen::Quaterniond ring_rot) {
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
	SmoothRotationCostFunctor(const Eigen::Quaterniond ring_rot) : ring_rot_(ring_rot) {}

	template <typename T> bool operator()(const T* const ring_rot, T* residual) const {
		Eigen::Map<const Eigen::Quaternion<T>> rot(ring_rot);

		Eigen::Quaternion<T> rot_diff = rot.conjugate() * ring_rot_.cast<T>();
		Eigen::AngleAxis<T> rot_diff_AA(rot_diff);
		T rot_error = rot_diff_AA.angle();

		Vector3<T> x_proj = rot * Vector3<T>(T(1), T(0), T(0));

		residual[0] = x_proj[0];// rot_error;
		return true;
	}
	void UpdateData(const Eigen::Quaterniond ring_rot) {
		ring_rot_ = ring_rot;
	}
private:
	Eigen::Quaterniond ring_rot_;
};

#if USE_JOINT
class PerFrameJointAngleCostFunctor
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
	PerFrameJointAngleCostFunctor(DataFrame data) : data_(data) {}

	template <typename T> bool operator()(const T* const joint_angles, const T* const wrist_offset, const T* const wrist_offset_rot, const T* const bone_lengths, const T* const cmc_alpha, T* residual) const {
		const T wrist_theta = joint_angles[0];
		const T wrist_phi = joint_angles[1];
		const T finger_theta = joint_angles[2];
		const T finger_phi = joint_angles[3];

		Eigen::Map<const Eigen::Vector3<T>> wrist_offset_(wrist_offset);
		Eigen::Map<const Eigen::Quaternion<T>> wrist_offset_rot_(wrist_offset_rot);
		const array<T, 2> bone_lengths_ = { bone_lengths[0], bone_lengths[1] };
		const T cmc_alpha_ = *cmc_alpha;
		Vector3<T> slack_pos = Vector3<T>(T(0), T(0), T(0));
		Quaternion<T> slack_rot = Quaternion<T>(T(1), T(0), T(0), T(0));
		hand_model_data<T> solved_model = compute_hand_model<T>(wrist_offset_, wrist_offset_rot_, bone_lengths_, cmc_alpha_, wrist_theta, wrist_phi, finger_theta, finger_phi, slack_pos, slack_rot);
		const Eigen::Vector3<T> pos = solved_model.ring_pos;
		const Eigen::Quaternion<T> rot = solved_model.ring_rot;

		const Eigen::Vector3<T> diff_pos = solved_model.ring_pos - data_.ring_pos.cast<T>();

		residual[0] = T(15) * diff_pos.norm();
		residual[1] = solved_model.ring_rot.angularDistance(data_.ring_q.cast<T>()) * T(180 / PI);
		
		return true;
	}


private:
	DataFrame data_;
};
#endif

class SlackCostFunctor
{
public:
	class SlackCostFunctor() {}

	template <typename T> bool operator()(const T* const slack_pos, const T* const slack_rot, T* residual) const {

		Eigen::Map<const Eigen::Vector3<T>> slack_pos_(slack_pos);
		Eigen::Map<const Eigen::Quaternion<T>> slack_rot_(slack_rot);

		AngleAxis<T> ang_ax = AngleAxis<T>(slack_rot_);

		//residual[0] = slack_pos_.norm() + T(.1);
		residual[0] = slack_pos_[0];
		residual[1] = slack_pos_[1];
		residual[2] = slack_pos_[2];
		residual[3] = ang_ax.angle() * T(180/PI);

		return true;
	}
};

double evaluate_frame(DataFrame data, per_frame_opt_parameters param, Eigen::Vector3d& ring_pos, Eigen::Quaterniond& ring_rot) {
#if USE_JOINT
	hand_model_data<double> solved_hand = compute_hand_model(param.hand_model, param.joint_angles, param.slack_pos, param.slack_rot);
	ring_pos = solved_hand.ring_pos;
	ring_rot = solved_hand.ring_rot;
#else
	ring_pos = param.ring_pos;
	ring_rot = param.ring_rot;
#endif
	Eigen::Vector3d diff = data.ring_pos - ring_pos;
	double min_diff = diff.norm();


	return min_diff;
}

void evaluate_frame(DataFrame data, per_frame_opt_parameters param) {
	Eigen::Vector3d ring_pos;
	Eigen::Quaterniond ring_rot;
	evaluate_frame(data, param, ring_pos, ring_rot);
}


void solve_per_frame(const CalibrationModel<double>& model, vector<DataFrame> rawData, per_frame_opt_parameters initial_values, string trial)//, vector<array<double, 4>> &angles, vector<hand_model_data<double>> &joints)
{
	std::cout << "Start Per Frame Opt" << endl;
	ceres::Problem frame_problem;

	per_frame_opt_parameters param;
	per_frame_opt_parameters param_lower;
	per_frame_opt_parameters param_upper;

#if USE_JOINT
	param = initial_values;
	param.slack_pos = Vector3d(0, 0, 0);
	param.slack_rot = Quaterniond(1, 0, 0, 0);


	param_lower.joint_angles = { -50 * PI / 180, -30 * PI / 180, -60 * PI / 180, -30 * PI / 180 };
	param_lower.slack_pos = Vector3d(-5, -5, -5);
	param_lower.slack_rot = Quaterniond(-1, -1, -1, -1);
	param_lower.hand_model.wrist_offset = Eigen::Vector3d(-10, -40, -40);
	param_lower.hand_model.wrist_offset_rot = Eigen::Quaterniond(-1, -1, -1, -1);
	param_lower.hand_model.bone_lengths = { 80, 10 };
	param_lower.hand_model.cmc_alpha = -10 * PI / 180;

	param_upper.joint_angles = { 50 * PI / 180, 40 * PI / 180, 0 * PI / 180, 30 * PI / 180 };
	param_upper.slack_pos = Vector3d(5, 5, 5);
	param_upper.slack_rot = Quaterniond(1, 1, 1, 1);
	param_upper.hand_model.wrist_offset = Eigen::Vector3d(10, -5, -10 );
	param_upper.hand_model.wrist_offset_rot = Eigen::Quaterniond(1, 1, 1, 1);
	param_upper.hand_model.bone_lengths = { 110, 40 };
	param_upper.hand_model.cmc_alpha = 10 * PI / 180;

	frame_problem.AddParameterBlock(param.joint_angles.data(), 4);
	frame_problem.AddParameterBlock(param.slack_pos.data(), 3);
	frame_problem.AddParameterBlock(param.slack_rot.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());
	frame_problem.AddParameterBlock(param.hand_model.wrist_offset.data(), 3);
	frame_problem.AddParameterBlock(param.hand_model.wrist_offset_rot.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());
	frame_problem.AddParameterBlock(param.hand_model.bone_lengths.data(), 2); // wristLen, fingerLen
	frame_problem.AddParameterBlock(&param.hand_model.cmc_alpha, 1); // wristLen, fingerLen

	for (unsigned int i = 0; i < param.joint_angles.size(); ++i) {
		frame_problem.SetParameterLowerBound(param.joint_angles.data(), i, param_lower.joint_angles[i]);
		frame_problem.SetParameterUpperBound(param.joint_angles.data(), i, param_upper.joint_angles[i]);
	}
	for (int i = 0; i < param.slack_pos.size(); ++i) {
		frame_problem.SetParameterLowerBound(param.slack_pos.data(), i, param_lower.slack_pos[i]);
		frame_problem.SetParameterUpperBound(param.slack_pos.data(), i, param_upper.slack_pos[i]);
	}
	for (int i = 0; i < param.slack_rot.coeffs().size(); ++i) {
		frame_problem.SetParameterLowerBound(param.slack_rot.coeffs().data(), i, param_lower.slack_rot.coeffs()[i]);
		frame_problem.SetParameterUpperBound(param.slack_rot.coeffs().data(), i, param_upper.slack_rot.coeffs()[i]);
	}
	for (int i = 0; i < param.hand_model.wrist_offset.size(); ++i) {
		frame_problem.SetParameterLowerBound(param.hand_model.wrist_offset.data(), i, param_lower.hand_model.wrist_offset[i]);
		frame_problem.SetParameterUpperBound(param.hand_model.wrist_offset.data(), i, param_upper.hand_model.wrist_offset[i]);
	}
	for (int i = 0; i < param.hand_model.wrist_offset_rot.coeffs().size(); ++i) {
		frame_problem.SetParameterLowerBound(param.hand_model.wrist_offset_rot.coeffs().data(), i, param_lower.hand_model.wrist_offset_rot.coeffs()[i]);
		frame_problem.SetParameterUpperBound(param.hand_model.wrist_offset_rot.coeffs().data(), i, param_upper.hand_model.wrist_offset_rot.coeffs()[i]);
	}
	for (unsigned int i = 0; i < param.hand_model.bone_lengths.size(); ++i) {
		frame_problem.SetParameterLowerBound(param.hand_model.bone_lengths.data(), i, param_lower.hand_model.bone_lengths[i]);
		frame_problem.SetParameterUpperBound(param.hand_model.bone_lengths.data(), i, param_upper.hand_model.bone_lengths[i]);
	}

	frame_problem.SetParameterLowerBound(&param.hand_model.cmc_alpha, 0, param_lower.hand_model.cmc_alpha);
	frame_problem.SetParameterUpperBound(&param.hand_model.cmc_alpha, 0, param_upper.hand_model.cmc_alpha);

	frame_problem.SetParameterBlockConstant(param.hand_model.wrist_offset.data());
	frame_problem.SetParameterBlockConstant(param.hand_model.bone_lengths.data());
	frame_problem.SetParameterBlockConstant(&param.hand_model.cmc_alpha);
	frame_problem.SetParameterBlockConstant(param.hand_model.wrist_offset_rot.coeffs().data());
	
	PerFrameCostFunctor* cost = new PerFrameCostFunctor(model, rawData[0], param.hand_model);
	ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PerFrameCostFunctor, NUM_RX * 3, 4, 3, 4, 3, 4, 2, 1>(cost);
	frame_problem.AddResidualBlock(cost_function, NULL, param.joint_angles.data(), param.slack_pos.data(), param.slack_rot.coeffs().data(), 
		param.hand_model.wrist_offset.data(), param.hand_model.wrist_offset_rot.coeffs().data(),
		param.hand_model.bone_lengths.data(), &param.hand_model.cmc_alpha);

	SlackCostFunctor* cost_slack = new SlackCostFunctor();
	ceres::CostFunction* cost_function_slack = new ceres::AutoDiffCostFunction<SlackCostFunctor, 4, 3, 4>(cost_slack);
	frame_problem.AddResidualBlock(cost_function_slack, NULL, param.slack_pos.data(), param.slack_rot.coeffs().data());

	
	SlippageCostFunctor* cost_slippage = new SlippageCostFunctor(param.hand_model);
	ceres::CostFunction* cost_function_slippage = new ceres::AutoDiffCostFunction<SlippageCostFunctor, 1, 3, 4, 2>(cost_slippage);
	
	SmoothJointsCostFunctor* cost_smooth = new SmoothJointsCostFunctor(param.hand_model);
	ceres::CostFunction* cost_function_smooth = new ceres::AutoDiffCostFunction<SmoothJointsCostFunctor, 4, 4, 3, 4, 3, 4, 2, 1>(cost_smooth);

	NeutralJointsCostFunctor* cost_neutral = new NeutralJointsCostFunctor();
	ceres::CostFunction* cost_function_neutral = new ceres::AutoDiffCostFunction<NeutralJointsCostFunctor, 4, 4>(cost_neutral);
#else
#if USE_POS
	param.ring_pos = rawData[0].ring_pos;
	param.ring_rot = rawData[0].ring_q;
#else
	param.ring_pos = { 0, -20, -100 };
	param.ring_rot = Eigen::Quaterniond(1, 0, 0, 0);
#endif

	param_lower.ring_pos = { -60, -130, -170 };
	param_lower.ring_rot = Eigen::Quaterniond(-1, -1, -1, -1);

	param_upper.ring_pos = { 85, 90, -50 };
	param_upper.ring_rot = Eigen::Quaterniond(1, 1, 1, 1);


	frame_problem.AddParameterBlock(param.ring_pos.data(), 3);
	frame_problem.AddParameterBlock(param.ring_rot.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());

	// set bounds on parameters
	for (unsigned int i = 0; i < 3; ++i) {
		frame_problem.SetParameterLowerBound(param.ring_pos.data(), i, param_lower.ring_pos[i]);
		frame_problem.SetParameterUpperBound(param.ring_pos.data(), i, param_upper.ring_pos[i]);
	}
	for (unsigned int i = 0; i < 4; ++i) {
		frame_problem.SetParameterLowerBound(param.ring_rot.coeffs().data(), i, param_lower.ring_rot.coeffs()[i]);
		frame_problem.SetParameterUpperBound(param.ring_rot.coeffs().data(), i, param_upper.ring_rot.coeffs()[i]);
	}


	HandModel null_model;
	PerFrameCostFunctor* cost = new PerFrameCostFunctor(model, rawData[0], null_model);
	ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PerFrameCostFunctor, NUM_RX * 3, 3, 4>(cost);
	frame_problem.AddResidualBlock(cost_function, NULL, param.ring_pos.data(), param.ring_rot.coeffs().data());

#endif
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.num_threads = 1;
	options.max_num_iterations = 100;



	ceres::Solver::Summary summary;

	Eigen::Vector3d ring_pos;
	Eigen::Quaterniond ring_rot;
	auto start_time = std::chrono::high_resolution_clock::now();
	for (unsigned int i = 0; i < rawData.size(); i++) {

#if USE_POS && !USE_JOINT
		if (rawData[i].ring_pos.x() < param_lower.ring_pos.x() || rawData[i].ring_pos.x() > param_upper.ring_pos.x() ||
			rawData[i].ring_pos.y() < param_lower.ring_pos.y() || rawData[i].ring_pos.y() > param_upper.ring_pos.y() ||
			rawData[i].ring_pos.z() < param_lower.ring_pos.z() || rawData[i].ring_pos.z() > param_upper.ring_pos.z()) {
			printf("skipping frame\n");
			continue;
		}
#endif
#if USE_JOINT
		cost_slippage->UpdateData(last_param.hand_model);
#endif
#if VERBOSE
		printf("Start of frame...\n");
		double error_pred = get_cost(ring_pos, ring_rot, rawData[i].sensors, model);
		std::printf("initial error: %f \n", ceres::sqrt(error_pred));
#endif

#if HALLUCINATE_SIM_DATA
		Vector3d field;
		Eigen::Vector3d sensor_1_pred = forward_model(rawData[i].ring_pos, rawData[i].ring_q, model.coil1, false, field);
		Eigen::Vector3d sensor_2_pred = forward_model(rawData[i].ring_pos, rawData[i].ring_q, model.coil2, false, field);
		Eigen::Vector3d sensor_3_pred = forward_model(rawData[i].ring_pos, rawData[i].ring_q, model.coil3, false, field);
		for (int j = 0; j < 3; j++) {
			rawData[i].sensors[j + 0] = sensor_1_pred[j] + ((rand() % 10000) - 5000) / 5000 * NOISE * sensor_1_pred[j];
			rawData[i].sensors[j + 3] = sensor_2_pred[j] + ((rand() % 10000) - 5000) / 5000 * NOISE * sensor_2_pred[j];
#if NUM_RX==3
			rawData[i].sensors[j + 6] = sensor_3_pred[j] + ((rand() % 10000) - 5000) / 5000 * NOISE * sensor_3_pred[j];
#endif
		}
#endif

		// Run the solver!
		cost->UpdateData(rawData[i], 0);

#if USE_JOINT
		cost_smooth->UpdateData(ring_pos, ring_rot);
#endif
#if CHEAT
		param.ring_pos = rawData[i].ring_pos;
		param.ring_rot = rawData[i].ring_q;
#endif

		ceres::Solve(options, &frame_problem, &summary);

		evaluate_frame(rawData[i], param, ring_pos, ring_rot);
#if VERBOSE
		if (min_diff > 1) {
			double error_pred = get_cost(ring_pos, ring_rot, rawData[i].sensors, model);
			double error_gt = get_cost(rawData[i].ring_pos, rawData[i].ring_q, rawData[i].sensors, model);
			std::printf("Error at predicted pos: %f \t Error at actual pos: %f\n", ceres::sqrt(error_pred), ceres::sqrt(error_gt));
			//std::cout << summary.FullReport() << "\n";

			Vector3d error = rawData[i].ring_pos - ring_pos;
			Quaterniond error_rot = rawData[i].ring_q * ring_rot.conjugate();
			AngleAxisd error_rot_angax = AngleAxisd(error_rot);
			AngleAxisd error_rot_angax_scaled = AngleAxisd(error_rot_angax.angle() * .1, error_rot_angax.axis());

			Vector3d next_step = ring_pos + error * 0.1;
			Quaterniond next_step_rot = Quaterniond(error_rot_angax_scaled) * ring_rot;
			double error_step = get_cost(next_step, next_step_rot, rawData[i].sensors, model);
			std::printf("Error at stepped pos: %f\n", ceres::sqrt(error_step));
		}
#endif
		//std::cout << summary.FullReport() << "\n";
		if (i % 100 == 0){
			auto now = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed = now - start_time;
			start_time = now;
			printf("%d - %f Hz\n", i, 100 / elapsed.count());
		}
		//if (min_diff > 12) {
		//	newFile = true;
		//	break;
		//}
		writePerFrameResults(CERES_PRED_DIRECTORY + "projected", trial, ring_pos, ring_rot, rawData[i].ring_pos, rawData[i].ring_q, param, newFile);
		newFile = false;
	}

	return;
}


#if USE_JOINT
void get_initial_conditions(DataFrame frame, per_frame_opt_parameters& param)
{
	std::cout << "Get initial conditions" << endl;
	ceres::Problem frame_problem;

	per_frame_opt_parameters param_lower;
	per_frame_opt_parameters param_upper;

	param_lower.joint_angles = { -90 * PI / 180, -40 * PI / 180, -80 * PI / 180, -40 * PI / 180 };
	param_lower.hand_model.wrist_offset = Eigen::Vector3d(-10, -40, -40);
	param_lower.hand_model.wrist_offset_rot = Eigen::Quaterniond(-1, -1, -1, -1);
	param_lower.hand_model.bone_lengths = { 75, 10 };
	param_lower.hand_model.cmc_alpha = -30 * PI / 180;

	param_upper.joint_angles = { 80 * PI / 180, 40 * PI / 180, 0 * PI / 180, 30 * PI / 180 };
	param_upper.hand_model.wrist_offset = Eigen::Vector3d(10, -5, -10);
	param_upper.hand_model.wrist_offset_rot = Eigen::Quaterniond(1, 1, 1, 1);
	param_upper.hand_model.bone_lengths = { 110, 30 };
	param_upper.hand_model.cmc_alpha = 30 * PI / 180;

	frame_problem.AddParameterBlock(param.joint_angles.data(), 4);
	frame_problem.AddParameterBlock(param.hand_model.wrist_offset.data(), 3); // rx_offset
	frame_problem.AddParameterBlock(param.hand_model.wrist_offset_rot.coeffs().data(), 4, new ceres::EigenQuaternionParameterization()); // rx_offset_rot
	frame_problem.AddParameterBlock(param.hand_model.bone_lengths.data(), 2); // wristLen, fingerLen
	frame_problem.AddParameterBlock(&param.hand_model.cmc_alpha, 1); // wristLen, fingerLen

	for (unsigned int i = 0; i < param.joint_angles.size(); ++i) {
		frame_problem.SetParameterLowerBound(param.joint_angles.data(), i, param_lower.joint_angles[i]);
		frame_problem.SetParameterUpperBound(param.joint_angles.data(), i, param_upper.joint_angles[i]);
	}
	for (int i = 0; i < param.hand_model.wrist_offset.size(); ++i) {
		frame_problem.SetParameterLowerBound(param.hand_model.wrist_offset.data(), i, param_lower.hand_model.wrist_offset[i]);
		frame_problem.SetParameterUpperBound(param.hand_model.wrist_offset.data(), i, param_upper.hand_model.wrist_offset[i]);
	}
	for (int i = 0; i < param.hand_model.wrist_offset_rot.coeffs().size(); ++i) {
		frame_problem.SetParameterLowerBound(param.hand_model.wrist_offset_rot.coeffs().data(), i, param_lower.hand_model.wrist_offset_rot.coeffs()[i]);
		frame_problem.SetParameterUpperBound(param.hand_model.wrist_offset_rot.coeffs().data(), i, param_upper.hand_model.wrist_offset_rot.coeffs()[i]);
	}
	for (unsigned int i = 0; i < param.hand_model.bone_lengths.size(); ++i) {
		frame_problem.SetParameterLowerBound(param.hand_model.bone_lengths.data(), i, param_lower.hand_model.bone_lengths[i]);
		frame_problem.SetParameterUpperBound(param.hand_model.bone_lengths.data(), i, param_upper.hand_model.bone_lengths[i]);
	}
	frame_problem.SetParameterLowerBound(&param.hand_model.cmc_alpha, 0, param_lower.hand_model.cmc_alpha);
	frame_problem.SetParameterUpperBound(&param.hand_model.cmc_alpha, 0, param_upper.hand_model.cmc_alpha);
	//frame_problem.SetParameterBlockConstant(param.joint_angles.data());
	frame_problem.SetParameterBlockConstant(param.hand_model.bone_lengths.data());
	frame_problem.SetParameterBlockConstant(&param.hand_model.cmc_alpha);
	frame_problem.SetParameterBlockConstant(param.hand_model.wrist_offset.data());
	frame_problem.SetParameterBlockConstant(param.hand_model.wrist_offset_rot.coeffs().data());
	PerFrameJointAngleCostFunctor* cost = new PerFrameJointAngleCostFunctor(frame);
	ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PerFrameJointAngleCostFunctor, 2, 4, 3, 4, 2, 1>(cost);
	frame_problem.AddResidualBlock(cost_function, NULL, param.joint_angles.data(), 
		param.hand_model.wrist_offset.data(), param.hand_model.wrist_offset_rot.coeffs().data(),
		param.hand_model.bone_lengths.data(), &param.hand_model.cmc_alpha);


	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.num_threads = 8;
	options.max_num_iterations = 300;

	ceres::Solver::Summary summary;

	ceres::Solve(options, &frame_problem, &summary);
	evaluate_frame(frame, param);

	return;
}
#endif

int main(int argc, char* argv[])
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	std::cout << "RUNNING PER FRAME OPT" << endl;
	CalibrationModel<double> calibration_model = {};
	std::cout << "STARTED PARSING DATA" << endl;
#if USE_DIPOLE
	calibration_model = parseCalibratedFile(CERES_DIRECTORY + "calibrate", "dipole", ' ');
	vector<DataFrame> data = parseCSVFile(MATLAB_DRECTORY + "dipole_data", TRIAL, ',');
#else
	HandModel hand_model = {};

	calibration_model = parseCalibratedFile(getenv("AURARING_DATA") + CERES_DIRECTORY + "calibrate", FLAGS_calibration, ' ');
#if USE_JOINT
	hand_model = parseHandModel(CERES_DIRECTORY + "handModel", HAND_MODEL);
	std::printf("Initial Values are:\n");
	std::printf("Bone Len: (%f, %f)\n", hand_model.bone_lengths[0], hand_model.bone_lengths[1]);
	std::printf("Wrist offset: (%f, %f, %f)\n", hand_model.wrist_offset[0], hand_model.wrist_offset[1], hand_model.wrist_offset[2]);
	std::printf("Wrist offset rot(w,x,y,z): (%f, %f, %f, %f)\n", hand_model.wrist_offset_rot.w(), hand_model.wrist_offset_rot.x(), hand_model.wrist_offset_rot.y(), hand_model.wrist_offset_rot.z());
	std::printf("CMC: %f\n", hand_model.cmc_alpha);
#endif
	vector<DataFrame> data = parseCSVFile(PROCESSED_DRECTORY + "resampled", FLAGS_trial, ' ', FLAGS_limit);
	for (unsigned int i = 0; i < data.size(); i++) {
		array<double, NUM_RX * 3> sensor_v;
		sensor_v[0] = data[i].sensors[0];
		sensor_v[1] = data[i].sensors[4];
		sensor_v[2] = data[i].sensors[2];
		sensor_v[3] = data[i].sensors[1];
		sensor_v[4] = data[i].sensors[5];
		sensor_v[5] = data[i].sensors[3];
		sensor_v[6] = data[i].sensors[6];
		sensor_v[7] = data[i].sensors[8];
		sensor_v[8] = data[i].sensors[7];
		data[i].sensors = sensor_v;
		data[i].ring_pos *= 1000;
	}
#endif

	cout << "FINISHED PARSING DATA" << endl;


	per_frame_opt_parameters initial_values;
#if USE_JOINT
	initial_values.hand_model.wrist_offset = hand_model.wrist_offset;
	initial_values.hand_model.wrist_offset_rot = hand_model.wrist_offset_rot;
	initial_values.hand_model.bone_lengths = hand_model.bone_lengths;
	initial_values.hand_model.cmc_alpha = hand_model.cmc_alpha;
	initial_values.hand_model.base_slack_pos = hand_model.base_slack_pos;
	initial_values.hand_model.base_slack_rot = hand_model.base_slack_rot;
	get_initial_conditions(data[0], initial_values);

#else
#endif
	solve_per_frame(calibration_model, data, initial_values, FLAGS_trial + "_" + FLAGS_calibration);
}

