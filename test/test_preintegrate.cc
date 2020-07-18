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

// accl, gyro
vector<vector<double>> imu_datas = {
    {-0.099134701513277898,0.14730578886832138,0.02722713633111154,8.1476917083333333,-0.37592158333333331,-2.4026292499999999},
    {-0.099134701513277898,0.14032447186034408,0.029321531433504733,8.033280791666666,-0.40861041666666664,-2.4026292499999999},
    {-0.098436569812480182,0.12775810124598494,0.037699111843077518,7.8861810416666662,-0.42495483333333334,-2.4353180833333332},
    {-0.10262536001726656,0.11588986233242347,0.045378560551852569,7.8289755833333325,-0.37592158333333331,-2.4680069166666665},
    {-0.1054178868204575,0.10821041362364843,0.05166174585903216,7.7145646666666661,-0.40861041666666664,-2.557901208333333},
    {-0.10890854532444616,0.097738438111682452,0.056548667764616284,7.6573592083333333,-0.43312704166666666,-2.6396232916666666},
    {-0.11030480872604163,0.085870199198121014,0.061435589670200401,7.5919815416666658,-0.46581587499999999,-2.7050009583333332},
    {-0.114493598930828,0.074700091985357306,0.060039326268604934,7.5511204999999997,-0.47398808333333331,-2.7458620000000002},
    {-0.11100294042683936,0.057246799465414,0.062133721370998138,7.5919815416666658,-0.51484912499999991,-2.8847895416666667},
    {-0.11728612573401893,0.046774823953448029,0.061435589670200401,7.6246703749999991,-0.48216029166666663,-2.9256505833333333},
};

int main(int argc, char** argv) {
    Vector3f accl, gyro;
    Vector3f accl_bias, gyro_bias;

    accl.setZero();
    gyro.setZero();

    accl_bias.setZero();
    gyro_bias.setZero();

    PreIntegrate* integ = new PreIntegrate(accl, gyro, accl_bias, gyro_bias);

    for (vector<double> &imu : imu_datas) {
        accl << imu[0], imu[1], imu[2];
        gyro << imu[3], imu[4], imu[5];

        integ->push_back(0.005, accl, gyro);
    }

    cout << "before gyro bias" << endl;
    cout << "p: " << integ->delta_p_.transpose() << endl;
    cout << "q: " << integ->delta_q_.coeffs().transpose() << endl;
    cout << "v: " << integ->delta_v_.transpose() << endl;

    Quaternionf delta_q = integ->delta_q_;
    Vector3f    delta_p = integ->delta_p_;
    Vector3f    delta_v = integ->delta_v_;

    Matrix3f Jqbw = integ->Jacobian_.block<3, 3>(J_OR, J_OBW).cast<float>();
    Matrix3f Jpba = integ->Jacobian_.block<3, 3>(J_OP, J_OBA).cast<float>();
    Matrix3f Jpbw = integ->Jacobian_.block<3, 3>(J_OP, J_OBW).cast<float>();
    Matrix3f Jvba = integ->Jacobian_.block<3, 3>(J_OV, J_OBA).cast<float>();
    Matrix3f Jvbw = integ->Jacobian_.block<3, 3>(J_OV, J_OBW).cast<float>();

    Vector3f new_gyro_bias(-0.00193261, 0.0482494, 0.0797451);
    Vector3f new_accl_bias(-0.025266,   0.136696,  0.075593);

    integ->reintegrate(new_accl_bias, new_gyro_bias);

    cout << "after gyro bias" << endl;
    cout << "p: " << integ->delta_p_.transpose() << endl;
    cout << "q: " << integ->delta_q_.coeffs().transpose() << endl;
    cout << "v: " << integ->delta_v_.transpose() << endl;


    Quaternionf q = delta_q*vec2quat<float>(Jpbw*new_gyro_bias);
    Vector3f    p = delta_p + Jpba*new_accl_bias + Jpbw*new_gyro_bias;
    Vector3f    v = delta_v + Jvba*new_accl_bias + Jvbw*new_gyro_bias;
    q.normalize();

    cout << "after gyro bias pre" << endl;
    cout << "p: " << p.transpose() << endl;
    cout << "q: " << q.coeffs().transpose() << endl;
    cout << "v: " << v.transpose() << endl;

    return 1;
}
