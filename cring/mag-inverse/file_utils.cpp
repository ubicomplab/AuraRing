#include "file_utils.h"
#include "settings.h"
#include <iostream>
#include <fstream>

vector<DataFrame> parseCSVFile(string fileName, string trial, char delimiter, unsigned int limit)
{
	string full_filename = fileName + "__" + trial + ".csv";
	ifstream file(full_filename);

	vector<DataFrame> data;
	unsigned int row = 0;
	std::string line;
	while (!file.eof()) {
		std::getline(file, line);
		if (line == "") {
			break;
		}
		std::stringstream iss(line);
		vector<double> parsed_line;
		while (!iss.eof())
		{
			std::string val;
			std::getline(iss, val, delimiter);
			parsed_line.push_back(stod(val));
		}
		/*if (parsed_line.size() < (9 + 3 + 4)) {
			break;
		}*/
		unsigned int i = 0;
		DataFrame frame;
		for (i = 0; i < 9; i++) {
			frame.sensors[i] = parsed_line[i];
		}
#if USE_POS
		frame.ring_pos = Eigen::Vector3d(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;

		frame.ring_q = Eigen::Quaterniond(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3]);
		i += 4;
#endif
		i += 3; // wrist pos
		i += 4; // wrist rot
#ifdef USE_PALM
		frame.palm_pos = Eigen::Vector3<double>(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;
		frame.palm_q = Eigen::Quaternion<double>(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3]);
		i += 4;
#endif
/*#ifdef USE_MARKERS
		vector<float> markers(parsed_line.begin() + i, parsed_line.end());
		frame.markers = markers;
#endif*/

		if (parsed_line.size() > i) {
			for (int j = 0; j < 4; j++) {
				frame.joint_angles[j] = parsed_line[i + j];
			}
		}
		data.push_back(frame);

		row++;
		if ((limit > 0) && (row > limit)) {
			break;
		}
	}

	if (data.size() == 0) {
		printf("File not found: %s\n", full_filename.c_str());
		cin.get();
		exit(1);
	}
	return data;
}


vector<DataFrame> parseProjectedCSVFile(string fileName, string trial, char delimiter, unsigned int limit)
{
	string full_filename = fileName + "__" + trial + ".csv";
	ifstream file(full_filename);

	vector<DataFrame> data;
	unsigned int row = 0;
	std::string line;
	while (!file.eof()) {
		std::getline(file, line);
		if (line == "") {
			break;
		}
		std::stringstream iss(line);
		vector<double> parsed_line;
		while (!iss.eof())
		{
			std::string val;
			std::getline(iss, val, delimiter);
			try {
				parsed_line.push_back(stod(val));
			}
			catch (std::invalid_argument&) {
				continue;
			}
			catch (std::out_of_range&) {
				break;
			}
		}

		unsigned int i = 0;
		DataFrame frame;

		frame.ring_pos = Eigen::Vector3d(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;

		frame.ring_q = Eigen::Quaterniond(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3]).conjugate();
		i += 4;


		data.push_back(frame);

		row++;
		if ((limit > 0) && (row > limit)) {
			break;
		}
	}
	return data;
}

CalibrationModel<double> parseCalibratedFile(string fileName, string trial, char delimiter = ' ')
{
	ifstream file(fileName + "__" + trial + ".csv");
	CalibrationModel<double> calibrated = {};
	int row = 0;
	std::string line;
	while (!file.eof()) {
		std::getline(file, line);
		if (line == "") {
			break;
		}
		std::stringstream iss(line);
		vector<float> parsed_line;
		while (!iss.eof())
		{
			std::string val;
			std::getline(iss, val, delimiter);
			parsed_line.push_back(stof(val));
		}
		if (parsed_line.size() < (1 + 3 + 3 + 3 + 3 + 3 + 4 + 3 + 4 + 3 + 3 + 4)) {
			break;
		}
		unsigned int i = 0;
		CoilCalibrationModel<double> frame;
		frame.global_gain = parsed_line[0];
		i += 1;
		frame.gain = Eigen::Vector3d(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;
		frame.per_channel_gain = Eigen::Vector3d(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;
		frame.bias = Eigen::Vector3d(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;
		frame.noise = Eigen::Vector3d(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;
		frame.sensor_offset = Eigen::Vector3d(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;
		frame.sensor_rot_offset = Eigen::Quaterniond(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3]);
		i += 4;
		frame.ring_offset = Eigen::Vector3d(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;
		frame.ring_rot_offset = Eigen::Quaterniond(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3]);
		i += 4;
		frame.crosstalk = { parsed_line[i], parsed_line[i + 1], parsed_line[i + 2] };
		i += 3;
		frame.base_sensor_offset = Eigen::Vector3d(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]);
		i += 3;
		frame.base_sensor_rot_offset = Eigen::Quaterniond(parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3]);
		i += 4;
		if (row == 0) {
			calibrated.coil1 = frame;
		}
		else if(row == 1)
		{
			calibrated.coil2 = frame;
		}
		else
		{
			calibrated.coil3 = frame;
		}
		row++;
	}
	return calibrated;
}

void writePerFrameResults(string fileName, string trial, const Eigen::Vector3d& ring_pos, const Eigen::Quaterniond& ring_rot, const Eigen::Vector3d& ring_pos_actual, const Eigen::Quaterniond& ring_rot_actual, const per_frame_opt_parameters& hand_model,
	bool newFile)
{
	// file pointer 
	fstream fout;

	if (newFile) {
		fout.open(fileName + "__" + trial + ".csv", ios::out);
		newFile = 0;
	}
	else {
		// opens an existing csv file or creates a new file. 
		fout.open(fileName + "__" + trial + ".csv", ios::app);
	}

	fout << ring_pos[0] << " " << ring_pos[1] << " " << ring_pos[2] << " ";
	fout << ring_rot.w() << " " << ring_rot.x() << " " << ring_rot.y() << " " << ring_rot.z() << " ";
	fout << ring_pos_actual[0] << " " << ring_pos_actual[1] << " " << ring_pos_actual[2] << " ";
	fout << ring_rot_actual.w() << " " << ring_rot_actual.x() << " " << ring_rot_actual.y() << " " << ring_rot_actual.z() << " ";
#if USE_JOINT
	fout << hand_model.joint_angles[0] << " " << hand_model.joint_angles[1] << " " << hand_model.joint_angles[2] << " " << hand_model.joint_angles[3] << " ";
	fout << hand_model.hand_model.wrist_offset[0] << " " << hand_model.hand_model.wrist_offset[1] << " " << hand_model.hand_model.wrist_offset[2] << " ";
	fout << hand_model.hand_model.wrist_offset_rot.w() << " " << hand_model.hand_model.wrist_offset_rot.x() << " " << hand_model.hand_model.wrist_offset_rot.y() << " " << hand_model.hand_model.wrist_offset_rot.z() << " ";
	fout << hand_model.hand_model.bone_lengths[0] << " " << hand_model.hand_model.bone_lengths[1] << " ";
	fout << hand_model.slack_pos[0] << " " << hand_model.slack_pos[1] << " " << hand_model.slack_pos[2] << " ";
	fout << hand_model.slack_rot.w() << " " << hand_model.slack_rot.x() << " " << hand_model.slack_rot.y() << " " << hand_model.slack_rot.z();
#endif
	fout << endl;
	fout.close();
}

void writeCalibration(string fileName, string trial, const CoilCalibrationModel<double>& coil, bool newFile)
{
	// file pointer 
	fstream fout;
	if (newFile) {
		fout.open(fileName + "__" + trial + ".csv", ios::out);
	}
	else {
		// opens an existing csv file or creates a new file. 
		fout.open(fileName + "__" + trial + ".csv", ios::app);
	}
	fout << coil.global_gain << " " << coil.gain[0] << " " << coil.gain[1] << " " << coil.gain[2] << " " << coil.per_channel_gain[0] << " ";
	fout << coil.per_channel_gain[1] << " " << coil.per_channel_gain[2] << " " << coil.bias[0] << " " << coil.bias[1] << " " << coil.bias[2] << " ";
	fout << coil.noise[0] << " " << coil.noise[1] << " " << coil.noise[2] << " " << coil.sensor_offset[0] << " " << coil.sensor_offset[1] << " " << coil.sensor_offset[2] << " ";
	fout << coil.sensor_rot_offset.w() << " " << coil.sensor_rot_offset.x() << " " << coil.sensor_rot_offset.y() << " " << coil.sensor_rot_offset.z() << " ";
	fout << coil.ring_offset[0] << " " << coil.ring_offset[1] << " " << coil.ring_offset[2] << " ";
	fout << coil.ring_rot_offset.w() << " " << coil.ring_rot_offset.x() << " " << coil.ring_rot_offset.y() << " " << coil.ring_rot_offset.z() << " ";
	fout << coil.crosstalk[0] << " " << coil.crosstalk[1] << " " << coil.crosstalk[2] << " ";
	fout << coil.base_sensor_offset[0] << " " << coil.base_sensor_offset[1] << " " << coil.base_sensor_offset[2] << " ";
	fout << coil.base_sensor_rot_offset.w() << " " << coil.base_sensor_rot_offset.x() << " " << coil.base_sensor_rot_offset.y() << " " << coil.base_sensor_rot_offset.z();
	fout << endl;

	fout.close();
}

void writeHandModel(string fileName, string trial, const HandModel& hand_model)
{
	// file pointer 
	fstream fout;

	// opens an existing csv file or creates a new file. 
	fout.open(fileName + "__" + trial + ".csv", ios::out);

	string name;
	fout << "wrist_len" << " " << "finger_len" << " " << "wrist_offset_x" << " " << "wrist_offset_y" << " " << "wrist_offset_z" << " " << "rx_rot_offset_k" << " " << "rx_rot_offset_x" << " " << "rx_rot_offset_y" << " " << "rx_rot_offset_z" << " "
		<< "palm_rot_offset_k" << " " << "palm_rot_offset_x" << " " << "palm_rot_offset_y" << " " << "palm_rot_offset_z" << " " 
		<< "palm_offset_x" << " " << "palm_offset_y" << " " << "palm_offset_z" << " "
		<< "base_slack_x" << " " << "base_slack_y" << " " << "base_slack_z" << " " << "base_slack_rot_w" << " " << "base_slack_rot_x" << " " << "base_slack_rot_y" << " " << "base_slack_rot_z" << " "
		<< "cmc_alpha" <<"\n";

	fout << hand_model.bone_lengths[0] << " " << hand_model.bone_lengths[1] << " ";
	fout << hand_model.wrist_offset[0] << " " << hand_model.wrist_offset[1] << " " << hand_model.wrist_offset[2] << " ";
	fout << hand_model.wrist_offset_rot.w() << " " << hand_model.wrist_offset_rot.x() << " " << hand_model.wrist_offset_rot.y() << " " << hand_model.wrist_offset_rot.z() << " ";
	fout << hand_model.palm_offset.x() << " " << hand_model.palm_offset.y() << " " << hand_model.palm_offset.z() << " ";
	fout << hand_model.palm_offset_rot.w() << " " << hand_model.palm_offset_rot.x() << " " << hand_model.palm_offset_rot.y() << " " << hand_model.palm_offset_rot.z() << " ";
	fout << hand_model.base_slack_pos[0] << " " << hand_model.base_slack_pos[1] << " " << hand_model.base_slack_pos[2] << " ";
	fout << hand_model.base_slack_rot.w() << " " << hand_model.base_slack_rot.x() << " " << hand_model.base_slack_rot.y() << " " << hand_model.base_slack_rot.z() << " ";
	fout << hand_model.cmc_alpha << "\n";

	fout.close();
}

void writeCSVFile(string fileName, string trial, vector<array<double, 4>> angles, vector<hand_model_data<double>> joints, vector<DataFrame> raw_data)
{
	// file pointer 
	fstream fout;
	hand_model_data<double> data;
	// opens an existing csv file or creates a new file. 
	fout.open(fileName + "__" + trial + ".csv", ios::out);

	string name;
	fout << "wrist_theta wrist_phi finger_theta finger_phi ";
	fout << "ring_x ring_y ring_z ring_qw ring_qx ring_qy ring_qz ";
	fout << "knuckle_x knuckle_y knuckle_z knuckle_qw knuckle_qx knuckle_qy knuckle_qz ";
	fout << "wrist_x wrist_y wrist_z wrist_qw wrist_qx wrist_qy wrist_qz ";
	fout << "palm_x palm_y palm_z palm_qw palm_qx palm_qy palm_qz\n";
	for (unsigned int i = 0; i < angles.size(); i++) {
		// Insert the data to file 
		fout << angles[i][0] << " " << angles[i][1] << " " << angles[i][2] << " " << angles[i][3];
		data = joints[i];
		fout << " " << data.ring_pos[0] << " " << data.ring_pos[1] << " " << data.ring_pos[2];
		fout << " " << data.ring_rot.w() << " " << data.ring_rot.x() << " " << data.ring_rot.y() << " " << data.ring_rot.z();
		fout << " " << data.knuckle_pos[0] << " " << data.knuckle_pos[1] << " " << data.knuckle_pos[2];
		fout << " " << data.knuckle_rot.w() << " " << data.knuckle_rot.x() << " " << data.knuckle_rot.y() << " " << data.knuckle_rot.z();
		fout << " " << data.wrist_pos[0] << " " << data.wrist_pos[1] << " " << data.wrist_pos[2];
		fout << " " << data.wrist_rot.w() << " " << data.wrist_rot.x() << " " << data.wrist_rot.y() << " " << data.wrist_rot.z();
		fout << " " << data.palm_pos[0] << " " << data.palm_pos[1] << " " << data.palm_pos[2];
		fout << " " << data.palm_rot.w() << " " << data.palm_rot.x() << " " << data.palm_rot.y() << " " << data.palm_rot.z();
		fout << "\n";
	}
	fout.close();
}


HandModel parseHandModel(string fileName, string trial) {
	ifstream file(fileName + "__" + trial + ".csv");
	if (file.fail()) {
		std::cout << "Hand model file does not exist";
		throw std::runtime_error("Hand model file does not exist");
	}
	HandModel data;
	int row = 0;
	std::string line;
	while (!file.eof()) {
		std::getline(file, line);
		if (line == "" || line[0] == 'w') {
			continue;
		}
		std::stringstream iss(line);
		vector<float> parsed_line;
		while (!iss.eof())
		{
			std::string val;
			std::getline(iss, val, ' ');
			parsed_line.push_back(stof(val));
		}
		unsigned int i = 0;
		data.bone_lengths = { parsed_line[i], parsed_line[i + 1] };
		i += 2;
		data.wrist_offset = { parsed_line[i], parsed_line[i + 1], parsed_line[i + 2] };
		i += 3;
		data.wrist_offset_rot = { parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3] };
		i += 4;
		data.palm_offset = { parsed_line[i], parsed_line[i + 1], parsed_line[i + 2]};
		i += 3;
		data.palm_offset_rot = { parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3] };
		i += 4;
		data.base_slack_pos = { parsed_line[i], parsed_line[i + 1], parsed_line[i + 2] };
		i += 3;
		data.base_slack_rot = { parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3] };
		i += 4;
		data.cmc_alpha = parsed_line[i];
	}
	return data;
}

vector<array<double, 4>> parseMatlab(string fileName, string trial) {
	ifstream file(fileName + "__" + trial + ".csv");
	vector<array<double, 4>> data;
	int row = 0;
	std::string line;
	while (!file.eof()) {
		std::getline(file, line);
		if (line == "") {
			break;
		}
		std::stringstream iss(line);
		vector<float> parsed_line;
		while (!iss.eof())
		{
			std::string val;
			std::getline(iss, val, ',');
			parsed_line.push_back(stof(val));
		}
		unsigned int i = 4;
		array<double, 4> frame;
		frame = { parsed_line[i], parsed_line[i + 1], parsed_line[i + 2], parsed_line[i + 3] };
		data.push_back(frame);
		if (row % 1000 == 0) {
			cout << row << "'s parsed" << endl;
		}
		row++;
	}
	return data;
}