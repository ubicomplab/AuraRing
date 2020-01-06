#include "settings.h"
#include "hand_models.h"
#include "mag_models.h"
#include "utils.h"
#include "file_utils.h"

#include <gflags/gflags.h>

#define RANDOM_SAMPLING 1

#define N 30000


DEFINE_string(trial, "", "trial to process");
DEFINE_string(calibration, "", "base calibration file");
DEFINE_string(suffix, "", "suffix");
DEFINE_bool(learn_sensor_offset, true, "learn sensor offset");
DEFINE_bool(learn_ring_offset, true, "learn ring offset");
DEFINE_bool(learn_channel_gain, true, "learn channel gain");
DEFINE_bool(learn_channel_noise, true, "learn noise");
DEFINE_bool(learn_channel_bias, true, "learn bias");
DEFINE_bool(learn_gain, true, "learn gain");
DEFINE_bool(learn_crosstalk, true, "learn crosstalk");

class GlobalCostFunctor
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
	GlobalCostFunctor(const array<DataFrame, N>& data, const CoilCalibrationModel<double>& initial_model, int sensor_offset) : data_(data), initial_model_(initial_model), sensor_offset_(sensor_offset){}

	template <typename T> bool operator()(const T* const p_global_gain, const T* const p_sensor_offset, const T* const p_sensor_rot_offset, 
		const T* const p_ring_offset, const T* const p_ring_rot_offset, const T* const p_per_channel_gain,
		const T* const p_noise, const T* const p_bias, const T* const p_gain,
		const T* const p_crosstalk, T* residual) const
	{
		Eigen::Map<const Eigen::Vector3<T>> sensor_offset(p_sensor_offset);
		Eigen::Map<const Eigen::Quaternion<T>> sensor_rot_offset(p_sensor_rot_offset);

		Eigen::Map<const Eigen::Vector3<T>> ring_offset(p_ring_offset);
		Eigen::Map<const Eigen::Quaternion<T>> ring_rot_offset(p_ring_rot_offset);
		Eigen::Map<const Eigen::Vector3<T>> per_channel_gain(p_per_channel_gain);
		Eigen::Map<const Eigen::Vector3<T>> noise(p_noise);
		Eigen::Map<const Eigen::Vector3<T>> bias(p_bias);
		Eigen::Map<const Eigen::Vector3<T>> gain(p_gain);
		

		CoilCalibrationModel<T> model = initial_model_.cast<T>();

		model.sensor_offset = sensor_offset;
		model.sensor_rot_offset = sensor_rot_offset;
		model.ring_offset = ring_offset;
		model.ring_rot_offset = ring_rot_offset;
		model.global_gain = *p_global_gain;
		model.per_channel_gain = per_channel_gain;
		model.crosstalk = { p_crosstalk[0], p_crosstalk[1], p_crosstalk[2] };

		model.noise = noise;
		model.bias = bias;
		model.gain = gain;

		print_calibration(model);

		Eigen::Array<T, Eigen::Dynamic, 3> mat_sensor_pred = Eigen::Array<T, Eigen::Dynamic, 3>::Zero(N, 3);
		Eigen::Array<T, Eigen::Dynamic, 3> mat_sensor = Eigen::Array<T, Eigen::Dynamic, 3>::Zero(N, 3);

		for (unsigned int frame_idx = 0; frame_idx < N; frame_idx++) {
			Eigen::Vector3<T> pos = data_[frame_idx].ring_pos.cast<T>();
			Eigen::Quaternion<T> rot = data_[frame_idx].ring_q.cast<T>();
			Vector3<T> field;
			Eigen::Vector3<T> sensor_pred = forward_model<T>(pos, rot, model, false, field); // Do not preserve sign

			for (unsigned int i = 0; i < 3; i++) {
				mat_sensor(frame_idx, i) = T(data_[frame_idx].sensors[i+ sensor_offset_]);
				mat_sensor_pred(frame_idx, i) = sensor_pred[i];
				if (ceres::abs(data_[frame_idx].sensors[i + sensor_offset_]) < 0.8)
				{
					T diff = T(data_[frame_idx].sensors[i + sensor_offset_]) - sensor_pred[i];
					residual[frame_idx * 3 + i] = diff;// / T(data_[frame_idx].sensors[i + sensor_offset_] + .001);
				}
				else
				{
					residual[frame_idx * 3 + i] = T(0);
				}
				//residual[i] += diff * diff;
			}
		}
		Eigen::Array<T, 1, 3> sensor_mean = mat_sensor.colwise().mean();
		Eigen::Array<T, 1, 3> sensor_pred_mean = mat_sensor_pred.colwise().mean();

		Eigen::Array<T, 1, 3> mat_sensor_std_dev = ((mat_sensor.rowwise() - sensor_mean).square().colwise().sum() / T(N - 1)).sqrt();
		Eigen::Array<T, 1, 3> mat_sensor_pred_std_dev = ((mat_sensor_pred.rowwise() - sensor_pred_mean).square().colwise().sum() / T(N - 1)).sqrt();

		// https://en.wikipedia.org/wiki/Pearson_correlation_coefficient
		Eigen::Array<T, 1, 3> r = ((mat_sensor.rowwise() - sensor_mean) * (mat_sensor_pred.rowwise() - sensor_pred_mean)).colwise().sum() / (
			(mat_sensor.rowwise() - sensor_mean).square().colwise().sum().sqrt() * (mat_sensor_pred.rowwise() - sensor_pred_mean).square().colwise().sum().sqrt());


		T cost = r.mean();


		printf("Rs: ");
		for (unsigned int kk = 0; kk < 3; kk++) {
			printf("%f ", D(r(kk)));
		}

		printf("\nCost: ");
		print_float(cost);

		return true;
	}

private:
	const array<DataFrame, N>& data_;
	int sensor_offset_;
	CoilCalibrationModel<double> initial_model_;
};

/*class ZeroFieldCostFunctor
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
	ZeroFieldCostFunctor(const array<double, 3>& sensors_at_zero, const CoilCalibrationModel<double>& initial_model, int sensor_offset) : sensors_at_zero_(sensors_at_zero), initial_model_(initial_model), sensor_offset_(sensor_offset) {}

	template <typename T> bool operator()(const T* const p_global_gain, const T* const p_per_channel_gain, const T* const p_crosstalk, T* residual) const
	{
		Eigen::Map<const Eigen::Vector3<T>> sensor_offset(p_sensor_offset);
		Eigen::Map<const Eigen::Quaternion<T>> sensor_rot_offset(p_sensor_rot_offset);

		Eigen::Map<const Eigen::Vector3<T>> ring_offset(p_ring_offset);
		Eigen::Map<const Eigen::Quaternion<T>> ring_rot_offset(p_ring_rot_offset);
		Eigen::Map<const Eigen::Vector3<T>> per_channel_gain(p_per_channel_gain);


		CoilCalibrationModel<T> model = initial_model_.cast<T>();

		model.sensor_offset = sensor_offset;
		model.sensor_rot_offset = sensor_rot_offset;
		model.ring_offset = ring_offset;
		model.ring_rot_offset = ring_rot_offset;
		model.global_gain = *p_global_gain;
		model.per_channel_gain = per_channel_gain;
		model.crosstalk = { p_crosstalk[0], p_crosstalk[1], p_crosstalk[2] };

		print_calibration(model);

		Eigen::Array<T, Eigen::Dynamic, 3> mat_sensor_pred = Eigen::Array<T, Eigen::Dynamic, 3>::Zero(N, 3);
		Eigen::Array<T, Eigen::Dynamic, 3> mat_sensor = Eigen::Array<T, Eigen::Dynamic, 3>::Zero(N, 3);

		for (unsigned int frame_idx = 0; frame_idx < N; frame_idx++) {
			Eigen::Vector3<T> pos = data_[frame_idx].ring_pos.cast<T>();
			Eigen::Quaternion<T> rot = data_[frame_idx].ring_q.cast<T>();
			Vector3<T> field;
			Eigen::Vector3<T> sensor_pred = forward_model<T>(pos, rot, model, false, field); // Do not preserve sign

			for (unsigned int i = 0; i < 3; i++) {
				mat_sensor(frame_idx, i) = T(data_[frame_idx].sensors[i + sensor_offset_]);
				mat_sensor_pred(frame_idx, i) = sensor_pred[i];
				if (ceres::abs(data_[frame_idx].sensors[i + sensor_offset_]) < 0.8)
				{
					T diff = T(data_[frame_idx].sensors[i + sensor_offset_]) - sensor_pred[i];
					residual[frame_idx * 3 + i] = diff;
				}
				else
				{
					residual[frame_idx * 3 + i] = T(0);
				}
				//residual[i] += diff * diff;
			}
		}
		Eigen::Array<T, 1, 3> sensor_mean = mat_sensor.colwise().mean();
		Eigen::Array<T, 1, 3> sensor_pred_mean = mat_sensor_pred.colwise().mean();

		Eigen::Array<T, 1, 3> mat_sensor_std_dev = ((mat_sensor.rowwise() - sensor_mean).square().colwise().sum() / T(N - 1)).sqrt();
		Eigen::Array<T, 1, 3> mat_sensor_pred_std_dev = ((mat_sensor_pred.rowwise() - sensor_pred_mean).square().colwise().sum() / T(N - 1)).sqrt();

		// https://en.wikipedia.org/wiki/Pearson_correlation_coefficient
		Eigen::Array<T, 1, 3> r = ((mat_sensor.rowwise() - sensor_mean) * (mat_sensor_pred.rowwise() - sensor_pred_mean)).colwise().sum() / (
			(mat_sensor.rowwise() - sensor_mean).square().colwise().sum().sqrt() * (mat_sensor_pred.rowwise() - sensor_pred_mean).square().colwise().sum().sqrt());


		T cost = r.mean();


		printf("Rs: ");
		for (unsigned int kk = 0; kk < 3; kk++) {
			printf("%f ", D(r(kk)));
		}

		printf("\nCost: ");
		print_float(cost);

		return true;
	}

private:
	const array<double, 3>& sensors_at_zero_;
	int sensor_offset_;
	CoilCalibrationModel<double> initial_model_;
};*/
/*
class GlobalGainCostFunctor
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
	GlobalGainCostFunctor(array<DataFrame, N> data, const CalibrationModel<double>& initial_model) : data_(data), initial_model_(initial_model) {}

	template <typename T> bool operator()(const T* const p_global_gain2, const T* const p_global_gain1, T* residual) const
	{
		CalibrationModel<T> model = initial_model_.cast<T>();

		model.coil1.global_gain = *p_global_gain1;
		model.coil2.global_gain = *p_global_gain2;

		print_calibration(model);

		for (unsigned int i = 0; i < 6; i++)
		{
			residual[i] = T(0);
		}


		for (unsigned int frame_idx = 0; frame_idx < N; frame_idx++) {
			Eigen::Vector3<T> pos = data_[frame_idx].ring_pos.cast<T>();
			Eigen::Quaternion<T> rot = data_[frame_idx].ring_q.cast<T>();
			Eigen::Vector3<T> sensor_1_pred = forward_model<T>(pos, rot, model.coil1, false); // Do not preserve sign
			Eigen::Vector3<T> sensor_2_pred = forward_model<T>(pos, rot, model.coil2, false); // Do not preserve sign

			for (unsigned int i = 0; i < 3; i++) {
				T diff1 = T(data_[frame_idx].sensors[i]) - sensor_1_pred[i];
				T diff2 = T(data_[frame_idx].sensors[i+3]) - sensor_2_pred[i];
				residual[i] += diff1 * diff1;
				residual[i+3] += diff2 * diff2;
			}
		}
		
		for (unsigned int i = 0; i < 6; i++)
		{
			residual[i] = ceres::sqrt(residual[i]);
		}
		return true;
	}

private:
	array<DataFrame, N> data_;
	CalibrationModel<double> initial_model_;
};

*/
void add_coil_parameters(ceres::Problem& problem, CoilCalibrationModel<double>& coil, const CoilCalibrationModel<double>& lower, const CoilCalibrationModel<double>& upper)
{
	problem.AddParameterBlock(coil.gain.data(), 3);
	
	problem.AddParameterBlock(coil.noise.data(), 3);
	problem.AddParameterBlock(coil.bias.data(), 3);
	problem.AddParameterBlock(coil.per_channel_gain.data(), 3);
	problem.AddParameterBlock(&coil.global_gain, 1);
	problem.AddParameterBlock(coil.sensor_offset.data(), 3);
	problem.AddParameterBlock(coil.sensor_rot_offset.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());
	problem.AddParameterBlock(coil.ring_offset.data(), 3);
	problem.AddParameterBlock(coil.ring_rot_offset.coeffs().data(), 4, new ceres::EigenQuaternionParameterization());
	problem.AddParameterBlock(coil.crosstalk.data(), 3);

	problem.SetParameterLowerBound(&coil.global_gain, 0, lower.global_gain);
	problem.SetParameterUpperBound(&coil.global_gain, 0, upper.global_gain);
	for (unsigned int j = 0; j < 3; ++j) {
		problem.SetParameterLowerBound(coil.gain.data(), j, lower.gain[j]);
		problem.SetParameterUpperBound(coil.gain.data(), j, upper.gain[j]);
		
		problem.SetParameterLowerBound(coil.noise.data(), j, lower.noise[j]);
		problem.SetParameterUpperBound(coil.noise.data(), j, upper.noise[j]);
		problem.SetParameterLowerBound(coil.bias.data(), j, lower.bias[j]);
		problem.SetParameterUpperBound(coil.bias.data(), j, upper.bias[j]);
		problem.SetParameterLowerBound(coil.per_channel_gain.data(), j, lower.per_channel_gain[j]);
		problem.SetParameterUpperBound(coil.per_channel_gain.data(), j, upper.per_channel_gain[j]);
		problem.SetParameterLowerBound(coil.sensor_offset.data(), j, lower.sensor_offset[j]);
		problem.SetParameterUpperBound(coil.sensor_offset.data(), j, upper.sensor_offset[j]);
		problem.SetParameterLowerBound(coil.ring_offset.data(), j, lower.ring_offset[j]);
		problem.SetParameterUpperBound(coil.ring_offset.data(), j, upper.ring_offset[j]);
		problem.SetParameterLowerBound(coil.crosstalk.data(), j, lower.crosstalk[j]);
		problem.SetParameterUpperBound(coil.crosstalk.data(), j, upper.crosstalk[j]);
	}
	for (unsigned int j = 0; j < 4; ++j) {
		problem.SetParameterLowerBound(coil.sensor_rot_offset.coeffs().data(), j, lower.sensor_rot_offset.coeffs()[j]);
		problem.SetParameterUpperBound(coil.sensor_rot_offset.coeffs().data(), j, upper.sensor_rot_offset.coeffs()[j]);
		problem.SetParameterLowerBound(coil.ring_rot_offset.coeffs().data(), j, lower.ring_rot_offset.coeffs()[j]);
		problem.SetParameterUpperBound(coil.ring_rot_offset.coeffs().data(), j, upper.ring_rot_offset.coeffs()[j]);
	}
}

std::array<DataFrame, N> sampled_data;

void solve_global_model(const vector<DataFrame>& rawData, CoilCalibrationModel<double>& coil_param, int sensor_offset)
{
	ceres::Problem global_problem;


	srand((unsigned int)time(NULL));
	int indices;
	for (int i = 0; i < N; i++) {
#if RANDOM_SAMPLING
		indices = rand() % rawData.size();
#else
		indices = i;
#endif
		sampled_data[i] = rawData[indices];
	}

	CoilCalibrationModel<double> global_param_lower;
	CoilCalibrationModel<double> global_param_upper;



	global_param_lower.global_gain = 0;
	global_param_lower.gain = Eigen::Vector3d(0, 0, 0);
	global_param_lower.per_channel_gain = Eigen::Vector3d(0, 0, 0);
	global_param_lower.noise = Eigen::Vector3d(0,0,0);
	global_param_lower.bias = Eigen::Vector3d(0,0,0);
	global_param_lower.sensor_offset = Eigen::Vector3d(-10, -10, -10);
	global_param_lower.sensor_rot_offset = Eigen::Quaterniond(-1, -1, -1, -1);
	global_param_lower.crosstalk = { -.5, -.5, -.5 };
	global_param_lower.ring_offset = Eigen::Vector3d(-10, -10, -10);
	global_param_lower.ring_rot_offset = Eigen::Quaterniond(-1, -1, -1, -1);


	global_param_upper.global_gain = 100;
	global_param_upper.gain = Eigen::Vector3d(1e3, 1e3, 1e3);
	global_param_upper.per_channel_gain = Eigen::Vector3d(1e3, 1e3, 1e3);
	global_param_upper.noise = Eigen::Vector3d(3, 3, 3);
	global_param_upper.bias = Eigen::Vector3d(0.2, 0.2, 0.2);
	global_param_upper.sensor_offset = Eigen::Vector3d(10, 10, 10);
	global_param_upper.sensor_rot_offset = Eigen::Quaterniond(1, 1, 1, 1);
	global_param_upper.crosstalk = { .5, .5, .5 };
	global_param_upper.ring_offset = Eigen::Vector3d(10, 10, 10);
	global_param_upper.ring_rot_offset = Eigen::Quaterniond(1, 1, 1, 1);



	add_coil_parameters(global_problem, coil_param, global_param_lower, global_param_upper);


	GlobalCostFunctor* cost = new GlobalCostFunctor(sampled_data, coil_param, sensor_offset);
	ceres::CostFunction* cost_function_global = new ceres::AutoDiffCostFunction<GlobalCostFunctor, 3*N, 1, 3, 4, 3, 4, 3, 3, 3, 3, 3>(cost);
	global_problem.AddResidualBlock(cost_function_global, NULL, &coil_param.global_gain, coil_param.sensor_offset.data(), coil_param.sensor_rot_offset.coeffs().data(),
		coil_param.ring_offset.data(), coil_param.ring_rot_offset.coeffs().data(), coil_param.per_channel_gain.data(), 
		coil_param.noise.data(), coil_param.bias.data(), coil_param.gain.data(),
		coil_param.crosstalk.data());

	if (!FLAGS_learn_sensor_offset) {
		global_problem.SetParameterBlockConstant(coil_param.sensor_offset.data());
		global_problem.SetParameterBlockConstant(coil_param.sensor_rot_offset.coeffs().data());
	}
	if (!FLAGS_learn_ring_offset) {
		global_problem.SetParameterBlockConstant(coil_param.ring_offset.data());
		global_problem.SetParameterBlockConstant(coil_param.ring_rot_offset.coeffs().data());
	}
	if (!FLAGS_learn_channel_gain) global_problem.SetParameterBlockConstant(coil_param.per_channel_gain.data());
	if (!FLAGS_learn_channel_noise) global_problem.SetParameterBlockConstant(coil_param.noise.data());
	if (!FLAGS_learn_channel_bias) global_problem.SetParameterBlockConstant(coil_param.bias.data());
	if (!FLAGS_learn_gain) global_problem.SetParameterBlockConstant(coil_param.gain.data());
	if (!FLAGS_learn_crosstalk) global_problem.SetParameterBlockConstant(coil_param.crosstalk.data());


	global_problem.SetParameterBlockConstant(&coil_param.global_gain);

	// Run the solver!
	cout << "Start Global Opt" << endl;
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 3000;
	//options.function_tolerance = 1e-8;
	options.num_threads = 8;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &global_problem, &summary);
	std::cout << summary.FullReport() << "\n";
	cout << "End Global Opt" << endl;

	print_calibration(coil_param);
}
/*
void solve_global_gain(vector<DataFrame> rawData, CalibrationModel<double>& global_param)
{
	ceres::Problem global_problem;

	std::array<DataFrame, N> sampled_data;

	srand((unsigned int)time(NULL));
	int indices;
	for (int i = 0; i < N; i++) {
#if RANDOM_SAMPLING
		indices = rand() % rawData.size();
#else
		indices = i;
#endif
		sampled_data[i] = rawData[indices];
	}

	CoilCalibrationModel<double> global_param_lower;
	CoilCalibrationModel<double> global_param_upper;



	double global_gain_lower = 0;
	double global_gain_upper = 100;


	global_problem.AddParameterBlock(&global_param.coil1.global_gain, 1);
	global_problem.SetParameterLowerBound(&global_param.coil1.global_gain, 0, global_gain_lower);
	global_problem.SetParameterUpperBound(&global_param.coil1.global_gain, 100, global_gain_upper);
	global_problem.AddParameterBlock(&global_param.coil2.global_gain, 1);
	global_problem.SetParameterLowerBound(&global_param.coil2.global_gain, 0, global_gain_lower);
	global_problem.SetParameterUpperBound(&global_param.coil2.global_gain, 100, global_gain_upper);

	GlobalGainCostFunctor* cost = new GlobalGainCostFunctor(sampled_data, global_param);
	ceres::CostFunction* cost_function_global = new ceres::AutoDiffCostFunction<GlobalGainCostFunctor, 6, 1, 1>(cost);
	global_problem.AddResidualBlock(cost_function_global, NULL, &global_param.coil1.global_gain, &global_param.coil2.global_gain);



	// Run the solver!
	cout << "Start Global Opt" << endl;
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 3000;
	options.num_threads = 8;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &global_problem, &summary);
	std::cout << summary.FullReport() << "\n";
	cout << "End Global Opt" << endl;

	print_calibration(global_param);
}
*/
int main(int argc, char* argv[])
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	google::InitGoogleLogging(argv[0]);

	cout << "STARTED PARSING DATA" << endl;
	vector<DataFrame> data1 = parseCSVFile(PROCESSED_DRECTORY + "resampled", FLAGS_trial, ' ');

	cout << "FINISHED PARSING DATA" << endl;
	for (unsigned int i = 0; i < data1.size(); i++) {
		array<double, NUM_RX * 3> sensor_v;
		sensor_v[0] = data1[i].sensors[0];
		sensor_v[1] = data1[i].sensors[4];
		sensor_v[2] = data1[i].sensors[2];
		sensor_v[3] = data1[i].sensors[1];
		sensor_v[4] = data1[i].sensors[5];
		sensor_v[5] = data1[i].sensors[3];
		sensor_v[6] = data1[i].sensors[6];
		sensor_v[7] = data1[i].sensors[8];
		sensor_v[8] = data1[i].sensors[7];
		data1[i].sensors = sensor_v;
		data1[i].ring_pos *= 1000;
	}

	CalibrationModel<double> global_param = parseCalibratedFile(CERES_DIRECTORY + "calibrate", FLAGS_calibration, ' ');

	solve_global_model(data1, global_param.coil1, 0);
	cout << "Coil 1 Global opt finished";
	//cin.get();
	/*global_param.coil2.ring_offset = global_param.coil1.ring_offset;
	global_param.coil2.ring_rot_offset = global_param.coil1.ring_rot_offset;
	global_param.coil3.ring_offset = global_param.coil1.ring_offset;
	global_param.coil3.ring_rot_offset = global_param.coil1.ring_rot_offset;*/
	solve_global_model(data1, global_param.coil2, 3);
	cout << "Coil 2 Global opt finished";
	//
	solve_global_model(data1, global_param.coil3, 6);
	cout << "Coil 3 Global opt finished";

	writeCalibration(CERES_DIRECTORY + "calibrate", FLAGS_trial + "_" + FLAGS_suffix, global_param.coil1, true);
	writeCalibration(CERES_DIRECTORY + "calibrate", FLAGS_trial + "_" + FLAGS_suffix, global_param.coil2, false);
	writeCalibration(CERES_DIRECTORY + "calibrate", FLAGS_trial + "_" + FLAGS_suffix, global_param.coil3, false);
	//solve_global_gain(data1, global_param);
	cin.get();
}

