#pragma once

#include <cmath>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace cv;
using namespace Eigen;

template<typename T>
Matrix<T, 3, 3> symmetricMatrix(const Matrix<T, 3, 1>& vec) {
    Matrix<T, 3, 3> ret;

    T x = vec(0, 0);
    T y = vec(1, 0);
    T z = vec(2, 0);

    ret <<  0, -z,  y, \
            z,  0, -x, \
           -y,  x,  0;

    return ret;
}

template<typename T>
Quaternion<T> vec2quat(const Matrix<T, 3, 1>& vec) {
    Quaternion<T> ret;

    T x = vec(0, 0);
    T y = vec(1, 0);
    T z = vec(2, 0);

    ret.w() = 1;
    ret.x() = x/2;
    ret.y() = y/2;
    ret.z() = z/2;
    
    return ret;
}
