#pragma once
#include "settings.h"
#include <vector>
#include <array>
#include "ceres/ceres.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "hand_models.h"

using namespace std;

#define PI 3.1415926535897932384626433832795028841971693993751058209749

template <typename T> T sgn(const T& val) {
	return T((T(0) < val) - (val < T(0)));
}

template <typename T> inline double D(const T& x);
template <> inline double D(const double& x) {
	return x;
}
template <size_t N> inline double D(const ceres::Jet<double, N>& x) {
	return x.a;
}

template <typename T>
inline void print_vec(Eigen::Vector3<T> vec) {
	printf("%f, %f, %f", D(vec[0]), D(vec[1]), D(vec[2]));
}
template <typename T>
inline void print_quat(Eigen::Quaternion<T> q) {
	printf("%f, %f, %f, %f", D(q.w()), D(q.x()), D(q.y()), D(q.z()));
}

template <typename T> inline void print_float(T x);
template <> inline void print_float(double x) {
	printf("%f\n", x);
}
template <size_t N> inline void print_float(ceres::Jet<double, N> x) {
	printf("%f\n", x.a);
}


template <typename T> inline void print_jacobian(T x);

template <typename T> inline void print_jacobian(T x);
template <> inline void print_jacobian(double x) {
	printf("%f", x);
}

template <size_t N> inline void print_jacobian(ceres::Jet<double, N> x) {
	printf("%f - %f %f %f %f\n", x.a, x.v[0], x.v[1], x.v[2], x.v[3]);
}


template <class T> inline void print_hand_model(T const* const wrist_offset, T const* const wrist_offset_rot, T const* const);

template <> inline void print_hand_model(double const* const wrist_offset, double const* const wrist_offset_rot, double const* const hand_model)
{
	printf("wrist_offset: (%f, %f, %f)\t rx_rot: (%f, %f, %f, %f)\t, wrist_len: %f, finger_len: %f\n", wrist_offset[0], wrist_offset[1], wrist_offset[2], wrist_offset_rot[0], wrist_offset_rot[1], wrist_offset_rot[2], wrist_offset_rot[3], hand_model[0], hand_model[1]);
}

template <size_t N> inline void print_hand_model(ceres::Jet<double, N> const* const wrist_offset, ceres::Jet<double, N> const* const wrist_offset_rot, ceres::Jet<double, N> const* const hand_model)
{
	printf("wrist_offset: (%f, %f, %f)\t rx_rot: (%f, %f, %f, %f)\t, wrist_len: %f, finger_len: %f\n", wrist_offset[0].a, wrist_offset[1].a, wrist_offset[2].a, wrist_offset_rot[0].a, wrist_offset_rot[1].a, wrist_offset_rot[2].a, wrist_offset_rot[3].a, hand_model[0].a, hand_model[1].a);
}

inline void print_hand_model(const HandModel &hand_model)
{
	printf("wrist_offset: (%f, %f, %f)\nrx_rot: (%f, %f, %f, %f)\npalm_offset: (%f, %f, %f)\npalm_rot: (%f, %f, %f, %f)\nwrist_len: %f, finger_len: %f, cmc_alpha: %f\n",
		hand_model.wrist_offset[0], hand_model.wrist_offset[1], hand_model.wrist_offset[2],
		hand_model.wrist_offset_rot.w(), hand_model.wrist_offset_rot.x(), hand_model.wrist_offset_rot.y(), hand_model.wrist_offset_rot.z(),
		hand_model.palm_offset[0], hand_model.palm_offset[1], hand_model.palm_offset[2],
		hand_model.palm_offset_rot.w(), hand_model.palm_offset_rot.x(), hand_model.palm_offset_rot.y(), hand_model.palm_offset_rot.z(),
		hand_model.bone_lengths[0], hand_model.bone_lengths[1], hand_model.cmc_alpha);

	printf("base_slack_pos: (%f, %f, %f)\nbase_slack_rot: (%f, %f, %f, %f)\n",
		hand_model.base_slack_pos[0], hand_model.base_slack_pos[1], hand_model.base_slack_pos[2],
		hand_model.base_slack_rot.w(), hand_model.base_slack_rot.x(), hand_model.base_slack_rot.y(), hand_model.base_slack_rot.z());
}