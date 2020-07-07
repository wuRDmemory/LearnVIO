#include "../../include/visual/project_factor.h"
#include "../../include/util/utils.h"
#include "../../include/util/tick.h"

Matrix2d VisualCost ::sqrt_info_;
double VisualCost::sum_t_ = 0;

bool PoseLocalParameter::Plus(const double* x, const double* delta, double* x_plus_delta) const {
    Vector3d    old_pwb(x[0], x[1], x[2]);
    Quaterniond old_qwb(x[6], x[3], x[4], x[5]);

    Map<const Vector3d> delta_pwb(delta);
    Map<const Vector3d> delta_qwb(delta+3);

    Vector3d    new_pwb = old_pwb + delta_pwb;
    Quaterniond new_qwb = old_qwb*vec2quat<double>(delta_qwb);
    new_qwb.normalize();

    x_plus_delta[0] = new_qwb.x();
    x_plus_delta[1] = new_qwb.y();
    x_plus_delta[2] = new_qwb.z();
    x_plus_delta[3] = new_qwb.w();

    return true;
}

bool PoseLocalParameter::ComputeJacobian(const double* x, double* jacobian) const {
    Map<Matrix<double, 4, 3, RowMajor>> transfer(jacobian);
    transfer.setIdentity();
    return true;
}


VisualCost::VisualCost(const Vector2d& ref_pt, const Vector2d& cur_pt) {
    ref_pt_ = ref_pt;
    cur_pt_ = cur_pt;    
}


bool VisualCost::Evaluate(double const* const* parameters, double* residuals, double** jacobian) const {
    Tick tic; 
    Vector3d    twbi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Quaterniond Qwbi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Vector3d    twbj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Quaterniond Qwbj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    double inv_depth = parameters[2][0];

    Matrix3d Rwbi = Qwbi.toRotationMatrix();
    Matrix3d Rwbj = Qwbj.toRotationMatrix();

    Vector3d Pci = ref_pt_/inv_depth;
    Vector3d Pbi = Rbc*Pci  + tbc;               // br_P = br_R_cr*cr_P + br_t_cr
    Vector3d Pw  = Rwbi*Pbi + twbi;              // w_P  = w_R_br*br_P + w_t_br
    Vector3d Pbj = Rwbj.inverse() *(Pw  - twbj); // bc_P = bc_R_w*(w_P - w_t_bc)
    Vector3d Pcj = Rbc.transpose()*(Pbj - tbc);  // cc_P = cc_R_bc*(bc_P - bc_t_cc)
    Vector2d pcj = Pcj.head<2>()/Pcj(2);

    Map<Vector2d> res(residuals);
    res = pcj - cur_pt_;
    res = sqrt_info_*res;

    if (jacobian) {
        Matrix<double, 2, 3, RowMajor> reduce;
        reduce << Pcj.x()/Pcj.z(), 0, -Pcj.x()/(Pcj.z()*Pcj.z()), 
                  0, Pcj.y()/Pcj.z(), -Pcj.y()/(Pcj.z()*Pcj.z());

        reduce = sqrt_info_*reduce;

        if (jacobian[0]) {  
            // partial of Ti
            Map<Matrix<double, 2, 7, RowMajor>> Jaco(jacobian[0]);

            Matrix<double, 3, 6, RowMajor> temp_J;
            temp_J.block<3, 3>(0, 0).setIdentity();
            temp_J.block<3, 3>(0, 3) = -Rwbi*symmetricMatrix(Pbi);

            Jaco.block<2, 6>(0, 0) = reduce*Rbc.transpose()*Rwbj.transpose()*temp_J;
            Jaco.rightCols(1).setZero();
        }

        if (jacobian[1]) {
            // partial of Tj
            Map<Matrix<double, 2, 7, RowMajor>> Jaco(jacobian[1]);

            Matrix<double, 3, 6, RowMajor> temp_J;
            temp_J.block<3, 3>(0, 0) = -Rwbj.transpose();
            temp_J.block<3, 3>(0, 3) = symmetricMatrix(Pbj);

            Jaco.block<2, 6>(0, 0) = reduce*Rbc.transpose()*temp_J;
            Jaco.rightCols(1).setZero();
        }

        if (jacobian[2]) {
            // partial of inverse_depth
            Map<Matrix<double, 2, 1, RowMajor>> Jaco(jacobian[2]);

            Matrix<double, 3, 1, RowMajor> temp_J;
            temp_J = -Pci/inv_depth; 

            Jaco = reduce*Rbc.transpose()*Rwbj.transpose()*Rwbi*Rbc*temp_J;
        }
    }
    sum_t_ += tic.delta_time();
    return true;
}

void VisualCost::check(double** parameters) {
    double*  res      = new double[2];
    double** Jacobian = new double*[2];
    Jacobian[0] = new double[2*7];
    Jacobian[1] = new double[2*7];

    Evaluate(parameters, res, Jacobian);
} 
