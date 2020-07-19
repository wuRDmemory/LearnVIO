#include <iostream>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "../include/visual/project_factor.h"
#include "../include/util/config.h"
#include "../include/util/utils.h"

using namespace std;
using namespace cv;
using namespace Eigen;

int main(int argc, char** argv) {
    Vector3f ref_pt(-0.057741, -0.039850, 1);
    Vector3f cur_pt(-0.052518, -0.087913, 1);

    VisualCost::Rbc <<  0.0148655, -0.999881, 0.0041403, \
                        0.999557,  0.0149672, 0.0257155, \
                        -0.0257744, 0.00375619, 0.999661;
    VisualCost::tbc << -0.0216401, -0.064677, 0.00981073;
    VisualCost::sqrt_info_.setIdentity();
    VisualCost::sum_t_ = 0;

    VisualCost* cost = new VisualCost(0, 1, ref_pt, cur_pt);

    double** parameters = new double*[3];
    parameters[0] = new double[POSE_SIZE]{0};
    parameters[1] = new double[POSE_SIZE]{0};
    parameters[2] = new double[1]{1.0};
    parameters[0][POSE_SIZE-1] = 1;
    parameters[1][POSE_SIZE-1] = 1;
    parameters[1][0] = 0.1;

    double* residual = new double[2];

    double** Jacobian = new double*[3];
    Jacobian[0] = new double[2*POSE_SIZE];
    Jacobian[1] = new double[2*POSE_SIZE];
    Jacobian[2] = new double[2*1];

    cost->Evaluate(parameters, residual, Jacobian);

    Map<Matrix<double, 2, 7, RowMajor>> Jrp1(Jacobian[0]);
    Map<Matrix<double, 2, 7, RowMajor>> Jrp2(Jacobian[1]);
    Map<Matrix<double, 2, 1>>           Jrdp(Jacobian[2]);
    Map<Matrix<double, 2, 1>>           res(residual);

    cout << "Jacobian1" << endl;
    cout << Jrp1 << endl;
    cout << Jrp2 << endl;
    cout << Jrdp << endl;

    cout << "residual1" << endl;
    cout << res.transpose() << endl;

    cout << "==============================" << endl;
    
    Map<Matrix<double, POSE_SIZE, 1>> pose1(parameters[0]); 
    Map<Matrix<double, POSE_SIZE, 1>> pose2(parameters[1]);
    Map<Matrix<double,         1, 1>> depth(parameters[2]);

    double** Jacobian_pert = new double*[3];
    Jacobian_pert[0] = new double[2*POSE_SIZE];
    Jacobian_pert[1] = new double[2*POSE_SIZE];
    Jacobian_pert[2] = new double[2*1];

    Map<Matrix<double, 2, 7>> Jrp1_pert(Jacobian_pert[0]);
    Map<Matrix<double, 2, 7>> Jrp2_pert(Jacobian_pert[1]);
    Map<Matrix<double, 2, 1>> Jrdp_pert(Jacobian_pert[2]);  

    double** parameters_pert = new double*[3];
    parameters_pert[0] = new double[POSE_SIZE];
    parameters_pert[1] = new double[POSE_SIZE];
    parameters_pert[2] = new double[1];
    
    Map<Matrix<double, 7, 1>> pose1_pert(parameters_pert[0]);
    Map<Matrix<double, 7, 1>> pose2_pert(parameters_pert[1]);
    Map<Matrix<double, 1, 1>> depth_pert(parameters_pert[2]);
    
    int i = 0, j = 0;
    const double e = 0.0001;

    vector<int> nums{POSE_SIZE-1, POSE_SIZE-1, 1};
    for (i = 0; i < 3; i++) {

        for (j = 0; j < nums[i]; j++) {

            double* perturb = new double[nums[i]]{0};
            perturb[j] = e;

            pose1_pert = pose1;
            pose2_pert = pose2;
            depth_pert = depth;

            if (i < 2) {
                Map<Quaterniond> q_pert(parameters_pert[i]+3);
                Map<Quaterniond> q(parameters[i]+3);
                q_pert = q*vec2quat<double>(Vector3d(perturb[3], perturb[4], perturb[5]));

                Map<Vector3d> p_pert(parameters_pert[i]);
                Map<Vector3d> p(parameters[i]);
                p_pert = p + Vector3d(perturb[0], perturb[1], perturb[2]);
            }
            else {
                depth_pert(0) += e;
            }

            double* res = new double[2]{0};
            cost->Evaluate(parameters_pert, res, NULL);

            Vector2d res_diff(res[0]-residual[0], res[1]-residual[1]);

            switch (i)
            {
            case 0:
                Jrp1_pert.col(j) = res_diff/e;
                break;
            case 1:
                Jrp2_pert.col(j) = res_diff/e;
                break; 
            case 2:
                Jrdp_pert.col(j) = res_diff/e;
                break;
            }
        }
    }

    cout << "Jacobian2" << endl;
    cout << Jrp1_pert << endl;
    cout << Jrp2_pert << endl;
    cout << Jrdp_pert << endl;

    return 1;
}
