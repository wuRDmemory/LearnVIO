#include <iostream>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "../include/util/log.h"
#include "../include/inertial/preintegrate.h"

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc, char** argv) {
    Vector3f accl, gyro;
    Vector3f accl_bias, gyro_bias;

    accl.setZero();
    gyro.setZero();
    PreIntegrate* integ = new PreIntegrate(accl, gyro, accl_bias, gyro_bias);

    for (;;) {
        accl.z() = theRNG().uniform(0.0, 10.0);

        integ->push_back(0.005, accl, gyro);
    }

    return 1;
}
