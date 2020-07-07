#pragma once

#include <iostream>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace cv;
using namespace Eigen;

void findEssential(const vector<cv::Point2f>& pts1, const vector<cv::Point2f>& pts2, cv::Mat &E21);
