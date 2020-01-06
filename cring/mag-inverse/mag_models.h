#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "utils.h"


using namespace std;
using namespace Eigen;

template <typename T>
struct CoilCalibrationModel
{
	T global_gain;
	Eigen::Vector3<T> noise;
	Eigen::Vector3<T> bias;
	Eigen::Vector3<T> gain;
	Eigen::Vector3<T> per_channel_gain;
	Eigen::Vector3<T> sensor_offset;
	Eigen::Quaternion<T> sensor_rot_offset;
	Eigen::Vector3<double> base_sensor_offset;
	Eigen::Quaternion<double> base_sensor_rot_offset;
	Eigen::Vector3<T> ring_offset;
	Eigen::Quaternion<T> ring_rot_offset;
	array<T, 3> crosstalk;

	template<typename T> CoilCalibrationModel<T> cast() const
	{
		CoilCalibrationModel<T> model;
		model.global_gain = T(global_gain);
		model.noise = noise.cast<T>();
		model.bias = bias.cast<T>();
		model.gain = gain.cast<T>();
		model.per_channel_gain = per_channel_gain.cast<T>();
		model.sensor_offset = sensor_offset.cast<T>();
		model.sensor_rot_offset = sensor_rot_offset.cast<T>();
		model.ring_offset = ring_offset.cast<T>();
		model.ring_rot_offset = ring_rot_offset.cast<T>();
		model.base_sensor_offset = base_sensor_offset;
		model.base_sensor_rot_offset = base_sensor_rot_offset;
		model.crosstalk = { T(crosstalk[0]), T(crosstalk[1]), T(crosstalk[2]) };
		return model;
	}
};

template <typename T>
struct CalibrationModel
{
	CoilCalibrationModel<T> coil1;
	CoilCalibrationModel<T> coil2;
	CoilCalibrationModel<T> coil3;

	template<typename T> CalibrationModel<T> cast() const
	{
		CalibrationModel<T> model;
		model.coil1 = coil1.cast<T>();
		model.coil2 = coil2.cast<T>();
		model.coil3 = coil3.cast<T>();
		return model;
	}
};



template <typename T>
Eigen::Vector3<T> dipole_model(const Eigen::Vector3<T>& pos) {
	T r = pos.norm();
	T r5 = pow(r, 5);
	T r2 = pow(r, 2);
	T Bx = pos[0] * pos[2] * T(3) / (r5);
	T By = pos[1] * pos[2] * T(3) / (r5);
	T Bz = (pos[2] * pos[2] * T(3) - r2) / (r5);
	return Eigen::Vector3<T>(Bx, By, Bz) * T(1e5);
}



template <typename T>
Eigen::Vector3<T> forward_model(const Eigen::Vector3<T>& ring_pos, const Eigen::Quaternion<T>& ring_rot, const CoilCalibrationModel<T>& model, bool preserve_sign, Vector3<T>& out_field)
{
	Eigen::Vector3<T> sensor_pos = model.base_sensor_offset + model.sensor_offset;
	Eigen::Quaternion<T> sensor_rot = model.sensor_rot_offset * model.base_sensor_rot_offset.cast<T>();
	Eigen::Vector3<T> ring_pos_adj = ring_pos + (ring_rot.conjugate() * model.ring_offset);
	//Eigen::Vector3<T> ring_pos_adj = ring_pos;
	Eigen::Quaternion<T> ring_rot_adj = ring_rot * model.ring_rot_offset;

	Eigen::Vector3<T> sensor_ring = ring_rot_adj * (sensor_pos - ring_pos_adj);
	Eigen::Vector3<T> field = ring_rot_adj.conjugate() * dipole_model(sensor_ring);


	Eigen::Vector3<T> field_sensor_frame = sensor_rot * field;
	out_field = field_sensor_frame;

	Eigen::Vector3<T> sensor;


	for (unsigned int i = 0; i < 3; i++)
	{
		sensor[i] = pow(field_sensor_frame[i] * model.global_gain / model.per_channel_gain[i], 2) + pow(model.noise[i], 2);
	}

	/*sensor[0] += pow(field_sensor_frame[1] * model.crosstalk[0] + field_sensor_frame[2] * model.crosstalk[1], 2);
	sensor[1] += pow(field_sensor_frame[0] * model.crosstalk[0] + field_sensor_frame[2] * model.crosstalk[2], 2);
	sensor[2] += pow(field_sensor_frame[0] * model.crosstalk[1] + field_sensor_frame[1] * model.crosstalk[2], 2);*/

	sensor[0] += pow(field_sensor_frame[1] * model.crosstalk[0], 2) + pow(field_sensor_frame[2] * model.crosstalk[1], 2);
	sensor[1] += pow(field_sensor_frame[0] * model.crosstalk[0], 2) + pow(field_sensor_frame[2] * model.crosstalk[2], 2);
	sensor[2] += pow(field_sensor_frame[0] * model.crosstalk[1], 2) + pow(field_sensor_frame[1] * model.crosstalk[2], 2);

	for (unsigned int i = 0; i < 3; i++)
	{
		sensor[i] = model.gain[i] * ceres::sqrt(sensor[i]) - model.bias[i];
		if (preserve_sign)
		{
			sensor[i] = sgn(field_sensor_frame[i]) * sensor[i];
		}
		if (!isnormal(D(sensor[i]))) {
			printf("complex result\n");
		}
	}

	return sensor;
}

template <typename T>
array<T, 9> forward_model(const Eigen::Vector3<T>& ring_pos, const Eigen::Quaternion<T>& ring_rot, const CalibrationModel<T>& model, bool preserve_sign)
{
	Vector3<T> field;
	Eigen::Vector3<T> sensor_pred1 = forward_model<T>(ring_pos, ring_rot, model.coil1, preserve_sign, field);
	Eigen::Vector3<T> sensor_pred2 = forward_model<T>(ring_pos, ring_rot, model.coil2, preserve_sign, field);
	Eigen::Vector3<T> sensor_pred3 = forward_model<T>(ring_pos, ring_rot, model.coil3, preserve_sign, field);
	array<T, 9> all_sensors;
	for (unsigned int i = 0; i < 3; i++) {
		all_sensors[i] = sensor_pred1[i];
		all_sensors[i + 3] = sensor_pred2[i];
		all_sensors[i + 6] = sensor_pred3[i];
	}
	return all_sensors;
}



//template <typename T> inline void print_calibration(CalibrationModel<T> model);
template <typename T>
inline void print_calibration(CalibrationModel<T> model) {
	/*printf("Gain 1: ");
	print_vec(model.gain_1);
	printf("\n");

	printf("Gain 2: ");
	print_vec(model.gain_2);
	printf("\n");


	printf("Bias: %f\n", D(model.bias));
	printf("Noise: %f\n", D(model.noise));*/

	printf("global gain: %f, %f\n", D(model.coil1.global_gain), D(model.coil2.global_gain));

	printf("sensor_1_offset: ");
	print_vec(model.coil1.sensor_offset);
	printf("\n");
	printf("sensor_2_offset: ");
	print_vec(model.coil2.sensor_offset);
	printf("\n");

	printf("sensor_1_rot_offset: ");
	print_quat(model.coil1.sensor_rot_offset);
	printf("\n");
	printf("sensor_2_rot_offset: ");
	print_quat(model.coil2.sensor_rot_offset);
	printf("\n");

	/*printf("crosstalk_1: (%f, %f, %f)\n", D(model.crosstalk_1[0]), D(model.crosstalk_1[1]), D(model.crosstalk_1[2]));
	printf("crosstalk_2: (%f, %f, %f)\n", D(model.crosstalk_2[0]), D(model.crosstalk_2[1]), D(model.crosstalk_2[2]));*/
}

template <typename T>
inline void print_calibration(CoilCalibrationModel<T> model) {
	printf("Noise: ");
	print_vec(model.noise);
	printf("\n");
	printf("Gain: ");
	print_vec(model.gain);
	printf("\n");
	printf("Bias: ");
	print_vec(model.bias);
	printf("\n");


	printf("global gain: %f\n", D(model.global_gain));

	printf("sensor_offset: ");
	print_vec(model.sensor_offset);
	printf("\n");

	printf("sensor_rot_offset: ");
	print_quat(model.sensor_rot_offset);
	printf("\n");

	printf("ring_offset: ");
	print_vec(model.ring_offset);
	printf("\n");

	printf("ring_rot_offset: ");
	print_quat(model.ring_rot_offset);
	printf("\n");


	printf("per_channel_gain: ");
	print_vec(model.per_channel_gain);
	printf("\n");

	printf("crosstalk: %f, %f, %f\n", D(model.crosstalk[0]), D(model.crosstalk[1]), D(model.crosstalk[2]));
	/*printf("crosstalk_1: (%f, %f, %f)\n", D(model.crosstalk_1[0]), D(model.crosstalk_1[1]), D(model.crosstalk_1[2]));
	printf("crosstalk_2: (%f, %f, %f)\n", D(model.crosstalk_2[0]), D(model.crosstalk_2[1]), D(model.crosstalk_2[2]));*/
}
