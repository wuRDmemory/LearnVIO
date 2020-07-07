#pragma once

#include <iostream>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "opencv/cxeigen.hpp"

#include "ceres/ceres.h"
#include "ceres/problem.h"
#include "ceres/rotation.h"

#include "feature.h"
#include "../inertial/preintegrate.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class FrameStruct {
public:
    PreIntegrate* preintegrate_;
    Matrix3d      Rcw_;   // world means camera0 frame
	Vector3d      Vbk_;
    Vector3d      tcw_;
	Vector3d      bias_g_;
	Vector3d      bias_a_;

    set<int>      feature_ids_;
    double        t_;

public:
    FrameStruct() = delete;
    FrameStruct(Image_Type& image, double t): t_(t), preintegrate_(nullptr) {
        Rcw_.setIdentity();
        tcw_.setZero();
		Vbk_.setZero();
		bias_g_.setZero();
		bias_a_.setZero();

        for (auto& pir : image) {
            feature_ids_.insert(pir.first);
        }
    }

	FrameStruct(const FrameStruct &frame) {
		preintegrate_ = nullptr;
		if (frame.preintegrate_ != nullptr) {
			preintegrate_ = new PreIntegrate();
			memcpy(preintegrate_, frame.preintegrate_, sizeof(PreIntegrate));
		}

		Rcw_    = frame.Rcw_;
		Vbk_    = frame.Vbk_;
		tcw_    = frame.tcw_;
		bias_g_ = frame.bias_g_;
		bias_a_ = frame.bias_a_;

    	feature_ids_ = frame.feature_ids_;
    	t_           = frame.t_;
	}
};

struct ReprojectionError3D {
	ReprojectionError3D(double observed_u, double observed_v)
		:observed_u(observed_u), observed_v(observed_v)
		{}

	template <typename T>
	bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const {
		T p[3];
		ceres::QuaternionRotatePoint(camera_R, point, p);
		p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
		T xp = p[0] / p[2];
    	T yp = p[1] / p[2];
    	residuals[0] = xp - T(observed_u);
    	residuals[1] = yp - T(observed_v);
    	return true;
	}

	static ceres::CostFunction* Create(const double observed_x,
	                                   const double observed_y)  {
	  return (new ceres::AutoDiffCostFunction<
	          ReprojectionError3D, 2, 4, 3, 3>(
	          	new ReprojectionError3D(observed_x,observed_y)));
	}

	double observed_u;
	double observed_v;
};

int computeRelativeRT(const vector<Point2f>& pts1, const vector<Point2f>& pts2, Matrix3d& Rcr, Vector3d& tcr);

int globalRefineBA(map<int, Feature*>& all_ftr, vector<FrameStruct*>& frames, int l);

int solveRTByPnP(int id1, map<int, Feature*>& all_ftr, set<int>& vis_ftr_id, Matrix3d& Rcr, Vector3d& tcr);

int trianglesTwoFrame(int id1, int id2, map<int, Feature*>& all_ftr, const vector<FrameStruct*>& all_frames);

Vector3d singleTriangle(Vector3d f1, Vector3d f2, const Matrix3d& R21, const Vector3d& t21);
