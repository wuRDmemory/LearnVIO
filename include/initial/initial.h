#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <map>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "../visual/globalsfm.h"
#include "../visual/feature.h"
#include "../inertial/preintegrate.h"

using namespace std;
using namespace cv;
using namespace Eigen;

int visualOnlyInit(map<int, Feature*>& all_ftr, vector<FrameStruct*>& all_frames);

int visualInertialAlign(vector<FrameStruct*>& frames, Matrix3d Rbc, Vector3d tbc, Vector3d &g_c0, double &s);