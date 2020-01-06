#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <array>

using namespace std;

struct HandModel
{
	Eigen::Vector3d wrist_offset;
	Eigen::Quaterniond wrist_offset_rot;
	array<double, 2> bone_lengths;						// wristLen,fingerLen
	Eigen::Vector3d palm_offset;
	Eigen::Quaterniond palm_offset_rot;
	Eigen::Vector3d base_slack_pos;
	Eigen::Quaterniond base_slack_rot;
	double cmc_alpha;
};


template <typename T> struct hand_model_data {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3<T> ring_pos;
	Eigen::Vector3<T> knuckle_pos;
	Eigen::Vector3<T> wrist_pos;
	Eigen::Vector3<T> palm_pos;
	Eigen::Quaternion<T> ring_rot;
	Eigen::Quaternion<T> palm_rot;
	Eigen::Quaternion<T> wrist_rot;
	Eigen::Quaternion<T> knuckle_rot;
};


template <typename T>
hand_model_data<T> compute_hand_model(const Eigen::Vector3<T>& wrist_offset, const Eigen::Quaternion<T>& wrist_offset_rot,
#ifdef USE_PALM
	const Eigen::Vector3<T>& palm_offset, const Eigen::Quaternion<T>& palm_offset_rot,
#endif
	const array<T, 2>& bone_lengths, const T& cmc_alpha,
	const T& wrist_theta, const T& wrist_phi, const T& finger_theta, const T& finger_phi,
	const Eigen::Vector3<T>& slack_pos, const Eigen::Quaternion<T>& slack_rot)
{
	hand_model_data<T> data;


	/*
	Device frame
	Wrist frame
	CMC frame
	Knuckle frame
	Finger frame = ring frame
	*/

	data.wrist_pos = wrist_offset;
	Eigen::Quaternion<T> wrist_frame = wrist_offset_rot;

	Eigen::AngleAxis<T> wrist_angle_axis_x(wrist_theta, Eigen::Vector3<T>(T(1), T(0), T(0)));
	Eigen::AngleAxis<T> wrist_angle_axis_y(wrist_phi, Eigen::Vector3<T>(T(0), T(1), T(0)));

	Eigen::Quaternion<T> cmc_frame = wrist_frame * wrist_angle_axis_x * wrist_angle_axis_y;

	Eigen::AngleAxis<T> cmc_rot(cmc_alpha, Eigen::Vector3<T>(T(0), T(0), T(-1)));
	Eigen::Quaternion<T> knuckle_frame = cmc_frame * cmc_rot;

	Eigen::AngleAxis<T> finger_angle_axis_x(finger_theta, Eigen::Vector3<T>(T(1), T(0), T(0)));
	Eigen::AngleAxis<T> finger_angle_axis_y(finger_phi, Eigen::Vector3<T>(T(0), T(1), T(0)));
	// This used to be cmc_frame * finger_angle_axis_x * finger_angle_axis_y;
	Eigen::Quaternion<T> ring_frame = knuckle_frame * finger_angle_axis_x * finger_angle_axis_y;

	data.knuckle_pos = data.wrist_pos + knuckle_frame * Eigen::Vector3<T>(T(0), T(0), -bone_lengths[0]);
	data.ring_pos = data.knuckle_pos + ring_frame * Eigen::Vector3<T>(T(0), T(0), -bone_lengths[1]);

	data.ring_pos += slack_pos;
	ring_frame *= slack_rot;

	data.ring_rot = ring_frame.conjugate();
	data.knuckle_rot = knuckle_frame.conjugate();
	data.wrist_rot = wrist_frame.conjugate();

#ifdef USE_PALM
	data.palm_rot = (knuckle_frame * palm_offset_rot).conjugate();
	data.palm_pos = data.wrist_pos + knuckle_frame * palm_offset;
#endif

	return data;
}

template <typename T>
hand_model_data<T> compute_hand_model(const HandModel& model, const T& wrist_theta, const T& wrist_phi, const T& finger_theta, const T& finger_phi, const Eigen::Vector3<T>& slack_pos, const Eigen::Quaternion<T>& slack_rot)
{
	Eigen::Vector3<T> rx_offset_T = model.wrist_offset.cast<T>();
	Eigen::Quaternion<T> rx_offset_rot_T = model.wrist_offset_rot.cast<T>();
	Eigen::Vector3<T> base_slack_pos_T = model.base_slack_pos.cast<T>();
	Eigen::Quaternion<T> base_slack_rot_T = model.base_slack_rot.cast<T>();
	std::array<T, 2> bone_lengths_T = { T(model.bone_lengths[0]), T(model.bone_lengths[1]) };
	T cmc_alpha_T = T(model.cmc_alpha);

#ifdef USE_PALM
	Eigen::Vector3<T> palm_offset_T = model.palm_offset.cast<T>();
	Eigen::Quaternion<T> palm_offset_rot_T = model.palm_offset_rot.cast<T>();
	return compute_hand_model<T>(rx_offset_T, rx_offset_rot_T, palm_offset_T, palm_offset_rot_T, bone_lengths_T, cmc_alpha_T, wrist_theta, wrist_phi, finger_theta, finger_phi, slack_pos + base_slack_pos_T, slack_rot * base_slack_rot_T);
#else
	return compute_hand_model<T>(rx_offset_T, rx_offset_rot_T, bone_lengths_T, cmc_alpha_T, wrist_theta, wrist_phi, finger_theta, finger_phi, slack_pos + base_slack_pos_T, slack_rot * base_slack_rot_T);
#endif
}

template <typename T>
hand_model_data<T> compute_hand_model(const HandModel& model, const array<T, 4>& joint_angles, const Eigen::Vector3<T>& slack_pos, const Eigen::Quaternion<T>& slack_rot)
{
	return compute_hand_model<T>(model, joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], slack_pos, slack_rot);
}
