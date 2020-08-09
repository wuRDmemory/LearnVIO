#pragma once

#include <iostream>
#include <algorithm>
#include <vector>
#include <unordered_map>

#include "Eigen/Dense"
#include "Eigen/Core"

#include "ceres/ceres.h"
#include "ceres/cost_function.h"

using namespace std;
using namespace Eigen;

struct ResidualBlockInfo
{
    ResidualBlockInfo(ceres::CostFunction *cost_function, ceres::LossFunction *loss_function, vector<double *> parameter_blocks, vector<int> drop_set)
        : cost_function_(cost_function), loss_function_(loss_function), parameter_blocks_(parameter_blocks), drop_set_(drop_set) {}

    void Evaluate();

    ceres::CostFunction *cost_function_;
    ceres::LossFunction *loss_function_;
    std::vector<double *> parameter_blocks_;
    std::vector<int>      drop_set_;

    vector<MatrixXd> jacobians_;
    VectorXd         residuals_;

    int localSize(int size) {
        return size == 7 ? 6 : size;
    }
};

class MarginalizationInfo {
public:
    ~MarginalizationInfo();

    int localSize(int size) const;
    int globalSize(int size) const;
    
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    
    void preMarginalize();
    void marginalize();
    
    vector<double*> 
    getParameterBlocks(unordered_map<long, double *> &addr_shift);

public:
    int m_, n_;
    int sum_block_size_;
    unordered_map<long, int>      parameter_block_size_;
    unordered_map<long, double *> parameter_block_data_;
    unordered_map<long, int>      parameter_block_idx_;

    vector<int>      keep_block_size_;
    vector<int>      keep_block_idx_ ;
    vector<double *> keep_block_data_;

    vector<ResidualBlockInfo *> factors_;

    MatrixXd linear_jacobian_;
    VectorXd linear_residual_;
    const double eps = 1e-8;
};

class MarginalFactor : public ceres::CostFunction {
public:
    MarginalizationInfo* margin_;

public:
    MarginalFactor(MarginalizationInfo* margin);

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const; 
};
