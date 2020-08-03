#include <iostream>
#include <vector>
#include <unordered_map>

#include "../include/util/utils.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
    Quaterniond q(0.99, 0.4, 0.5, 0);
    q.normalize();

    Matrix4d left_Q  = quatLeftMult(q);
    Matrix4d right_Q = quatRightMult(q);

    Matrix4d left_Q1  = quatLeftMult1(q);
    Matrix4d right_Q1 = quatRightMult1(q);

    cout << left_Q - left_Q1  << endl << endl;
    
    cout << right_Q - right_Q1  << endl;
    return 1;
}