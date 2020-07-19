#include "../../include/visual/project_factor.h"
#include "../../include/util/utils.h"
#include "../../include/util/tick.h"

bool PoseLocalParameter::Plus(const double* x, const double* delta, double* x_plus_delta) const {
    Map<const Vector3d>    old_pwb(x);
    Map<const Quaterniond> old_qwb(x+3);

    Map<const Vector3d> delta_pwb(delta);
    Map<const Vector3d> delta_qwb(delta+3);

    Map<Vector3d>    new_pwb(x_plus_delta);
    Map<Quaterniond> new_qwb(x_plus_delta+3);

    new_pwb = old_pwb + delta_pwb;
    new_qwb = old_qwb * vec2quat<double>(delta_qwb);
    new_qwb.normalize();

    FILE* fp = fopen("./delta.txt", "a");
    fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", delta[0], delta[1], delta[2], delta[3], delta[4], delta[5]);
    fclose(fp);

    return true;
}

bool PoseLocalParameter::ComputeJacobian(const double* x, double* jacobian) const {
    Map<Matrix<double, 7, 6, RowMajor>> transfer(jacobian);
    transfer.topRows<6>().setIdentity();
    transfer.bottomRows<1>().setZero();
    return true;
}

Matrix2d VisualCost::sqrt_info_;
double VisualCost::sum_t_ = 0;

Matrix3d VisualCost::Rbc;
Vector3d VisualCost::tbc;

VisualCost::VisualCost(int i, int j, const Vector3f& ref_pt, const Vector3f& cur_pt) {
    i_ = i;
    j_ = j;
    ref_pt_ = ref_pt.cast<double>();
    cur_pt_ = cur_pt.cast<double>();

    FILE* fp = fopen("./visual_factor.txt", "w");
    fclose(fp);
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
    Vector3d Pbi = Rbc*Pci  + tbc;                // br_P = br_R_cr*cr_P + br_t_cr
    Vector3d Pw  = Rwbi*Pbi + twbi;               // w_P  = w_R_br*br_P + w_t_br
    Vector3d Pbj = Rwbj.transpose()*(Pw  - twbj); // bc_P = bc_R_w*(w_P - w_t_bc)
    Vector3d Pcj = Rbc.transpose()*(Pbj - tbc);   // cc_P = cc_R_bc*(bc_P - bc_t_cc)
    Vector2d pcj = Pcj.head<2>()/Pcj(2);

    Map<Vector2d> res(residuals);
    res = pcj - cur_pt_.head<2>();
    res = sqrt_info_*res;

    FILE* fp = fopen("./visual_factor.txt", "a");
    fprintf(fp, "%d->%d  %lf, %lf, %lf, %lf, %lf, %lf\n", i_, j_, res(0), res(1), ref_pt_(0), ref_pt_(1), cur_pt_(0), cur_pt_(1));
    fclose(fp);

    if (jacobian) {
        Matrix<double, 2, 3, RowMajor> reduce;
        reduce << 1.0/Pcj.z(),           0, -Pcj.x()/(Pcj.z()*Pcj.z()), 
                            0, 1.0/Pcj.z(), -Pcj.y()/(Pcj.z()*Pcj.z());

        reduce = sqrt_info_*reduce;

         if (jacobian[0]) {
            // partial of Ti
            Map<Matrix<double, 2, 7, RowMajor>> Jaco(jacobian[0]);

            Matrix<double, 3, 6, RowMajor> temp_J;
            temp_J.leftCols<3>().setIdentity();
            temp_J.rightCols<3>() = -Rwbi*symmetricMatrix<double>(Pbi);

            Jaco.leftCols<6>() = reduce*Rbc.transpose()*Rwbj.transpose()*temp_J;
            Jaco.rightCols<1>().setZero();

            // Matrix<double, 2, 7> Jaco_temp = Jaco; 
            // cout << Jaco_temp << endl << endl;
        }

        if (jacobian[1]) {
            // partial of Tj
            Map<Matrix<double, 2, 7, RowMajor>> Jaco(jacobian[1]);

            Matrix<double, 3, 6, RowMajor> temp_J;
            temp_J.leftCols<3>()  = -Rwbj.transpose();
            temp_J.rightCols<3>() = symmetricMatrix<double>(Pbj);

            Jaco.leftCols<6>() = reduce*Rbc.transpose()*temp_J;
            Jaco.rightCols<1>().setZero();
            
            // Matrix<double, 2, 7> Jaco_temp = Jaco; 
            // cout << Jaco_temp << endl << endl;
        }

        if (jacobian[2]) {
            // partial of inverse_depth
            Map<Vector2d> Jaco(jacobian[2]);

            Vector3d temp_J;
            temp_J = -Pci/inv_depth; 

            Jaco = reduce*Rbc.transpose()*Rwbj.transpose()*Rwbi*Rbc*temp_J;
            
            // Vector2d Jaco_temp = Jaco; 
            // cout << Jaco_temp << endl << endl;
        }
    }
    sum_t_ += tic.delta_time();
    return true;
}

