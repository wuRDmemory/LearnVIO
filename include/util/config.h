#pragma once

#include <iostream>
#include <vector>

using namespace std;

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

extern int FEN_WINDOW_SIZE;

bool readParameters(string config_path, string vins_path);
