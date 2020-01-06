#pragma once
#include <string>
#include <stdlib.h>

using namespace std;

#define USE_JOINT				0
#define APPLY_SIGN_CORRECTION	0
#define HALLUCINATE_SIGN		0
//#define USE_MARKERS				0
#define NUM_RX 3
#define USE_POS					1
//#define USE_PALM				0


static const std::string get_home_dir() {
	char* auraring_dir;
	size_t auraring_dir_len;
	errno_t auraring_dir_err = _dupenv_s(&auraring_dir, &auraring_dir_len, "AURARING_DATA");
	if (auraring_dir_err) {
		throw std::runtime_error("Could not find AURARING_DATA environment variable");
	}
	return std::string(auraring_dir);
}
const string HOME_DIR = get_home_dir();

const string CERES_DIRECTORY = HOME_DIR + R"(/ceres/)";
const string CERES_PRED_DIRECTORY = HOME_DIR + R"(/ceres/)";
const string PROCESSED_DRECTORY = HOME_DIR + R"(/processed/)";
const string MATLAB_DRECTORY = HOME_DIR + R"(/MATLAB/)";