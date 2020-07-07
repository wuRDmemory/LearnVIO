#pragma once

#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "opencv/cxeigen.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;

extern string CONFIG_PATH;
extern string IMAGE_TOPIC;
extern string IMU_TOPIC;
extern vector<string> CAM_NAMES;
extern string FISHEYE_MASK;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

extern float ACCL_N, GYRO_N;
extern float ACCL_BIAS_N, GYRO_BIAS_N;
extern float INIT_DEPTH;

extern int FEN_WINDOW_SIZE;

extern vector<Matrix3d> Rics;
extern vector<Vector3d> tics;

bool readParameters(string config_path, string vins_path);
