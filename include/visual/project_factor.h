#pragma once

#include <iostream>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "ceres/sized_cost_function.h"
#include "ceres/local_parameterization.h"

#include "../util/tick.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class PoseLocalParameter : public ceres::LocalParameterization {
public:
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;

    virtual bool ComputeJacobian(const double* x, double* jacobian) const;

    virtual int GlobalSize() const { return 4; }
    
    virtual int LocalSize() const  { return 3; }
};

class VisualCost : public ceres::SizedCostFunction<2, 7, 7, 1> {
public:
    static Matrix2d sqrt_info_;
    static double   sum_t_;

    static Matrix3d Rbc;
    static Vector3d tbc;

private:
    double focus_length_;
    Vector2d ref_pt_;
    Vector2d cur_pt_;

public:
    VisualCost(const Vector2d& ref_pt, const Vector2d& cur_pt);
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobian) const;
    void check(double **parameters);
};
