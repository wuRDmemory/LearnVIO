#include "../../include/visual/globalsfm.h"
#include "../../include/util/utils.h"
#include "../../include/util/config.h"
#include "../../include/visual/globalsfm.h"
#include "../../include/visual/project_factor.h"

namespace cv {
    void decomposeEssentialMat( InputArray _E, OutputArray _R1, OutputArray _R2, OutputArray _t )
    {

        Mat E = _E.getMat().reshape(1, 3);
        CV_Assert(E.cols == 3 && E.rows == 3);

        Mat D, U, Vt;
        SVD::compute(E, D, U, Vt);

        if (determinant(U) < 0) U *= -1.;
        if (determinant(Vt) < 0) Vt *= -1.;

        Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
        W.convertTo(W, E.type());

        Mat R1, R2, t;
        R1 = U * W * Vt;
        R2 = U * W.t() * Vt;
        t = U.col(2) * 1.0;

        R1.copyTo(_R1);
        R2.copyTo(_R2);
        t.copyTo(_t);
    }

    int recoverPose( InputArray E, InputArray _points1, InputArray _points2, InputArray _cameraMatrix,
                         OutputArray _R, OutputArray _t, InputOutputArray _mask)
    {

        Mat points1, points2, cameraMatrix;
        _points1.getMat().convertTo(points1, CV_64F);
        _points2.getMat().convertTo(points2, CV_64F);
        _cameraMatrix.getMat().convertTo(cameraMatrix, CV_64F);

        int npoints = points1.checkVector(2);
        CV_Assert( npoints >= 0 && points2.checkVector(2) == npoints &&
                                  points1.type() == points2.type());

        CV_Assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3 && cameraMatrix.channels() == 1);

        if (points1.channels() > 1)
        {
            points1 = points1.reshape(1, npoints);
            points2 = points2.reshape(1, npoints);
        }

        double fx = cameraMatrix.at<double>(0,0);
        double fy = cameraMatrix.at<double>(1,1);
        double cx = cameraMatrix.at<double>(0,2);
        double cy = cameraMatrix.at<double>(1,2);

        points1.col(0) = (points1.col(0) - cx) / fx;
        points2.col(0) = (points2.col(0) - cx) / fx;
        points1.col(1) = (points1.col(1) - cy) / fy;
        points2.col(1) = (points2.col(1) - cy) / fy;

        points1 = points1.t();
        points2 = points2.t();

        Mat R1, R2, t;
        decomposeEssentialMat(E, R1, R2, t);
        Mat P0 = Mat::eye(3, 4, R1.type());
        Mat P1(3, 4, R1.type()), P2(3, 4, R1.type()), P3(3, 4, R1.type()), P4(3, 4, R1.type());
        P1(Range::all(), Range(0, 3)) = R1 * 1.0; P1.col(3) = t * 1.0;
        P2(Range::all(), Range(0, 3)) = R2 * 1.0; P2.col(3) = t * 1.0;
        P3(Range::all(), Range(0, 3)) = R1 * 1.0; P3.col(3) = -t * 1.0;
        P4(Range::all(), Range(0, 3)) = R2 * 1.0; P4.col(3) = -t * 1.0;

        // Do the cheirality check.
        // Notice here a threshold dist is used to filter
        // out far away points (i.e. infinite points) since
        // there depth may vary between postive and negtive.
        double dist = 50.0;
        Mat Q;
        triangulatePoints(P0, P1, points1, points2, Q);
        Mat mask1 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask1 = (Q.row(2) < dist) & mask1;
        Q = P1 * Q;
        mask1 = (Q.row(2) > 0) & mask1;
        mask1 = (Q.row(2) < dist) & mask1;

        triangulatePoints(P0, P2, points1, points2, Q);
        Mat mask2 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask2 = (Q.row(2) < dist) & mask2;
        Q = P2 * Q;
        mask2 = (Q.row(2) > 0) & mask2;
        mask2 = (Q.row(2) < dist) & mask2;

        triangulatePoints(P0, P3, points1, points2, Q);
        Mat mask3 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask3 = (Q.row(2) < dist) & mask3;
        Q = P3 * Q;
        mask3 = (Q.row(2) > 0) & mask3;
        mask3 = (Q.row(2) < dist) & mask3;

        triangulatePoints(P0, P4, points1, points2, Q);
        Mat mask4 = Q.row(2).mul(Q.row(3)) > 0;
        Q.row(0) /= Q.row(3);
        Q.row(1) /= Q.row(3);
        Q.row(2) /= Q.row(3);
        Q.row(3) /= Q.row(3);
        mask4 = (Q.row(2) < dist) & mask4;
        Q = P4 * Q;
        mask4 = (Q.row(2) > 0) & mask4;
        mask4 = (Q.row(2) < dist) & mask4;

        mask1 = mask1.t();
        mask2 = mask2.t();
        mask3 = mask3.t();
        mask4 = mask4.t();

        // If _mask is given, then use it to filter outliers.
        if (!_mask.empty())
        {
            Mat mask = _mask.getMat();
            CV_Assert(mask.size() == mask1.size());
            bitwise_and(mask, mask1, mask1);
            bitwise_and(mask, mask2, mask2);
            bitwise_and(mask, mask3, mask3);
            bitwise_and(mask, mask4, mask4);
        }
        if (_mask.empty() && _mask.needed())
        {
            _mask.create(mask1.size(), CV_8U);
        }

        CV_Assert(_R.needed() && _t.needed());
        _R.create(3, 3, R1.type());
        _t.create(3, 1, t.type());

        int good1 = countNonZero(mask1);
        int good2 = countNonZero(mask2);
        int good3 = countNonZero(mask3);
        int good4 = countNonZero(mask4);

        if (good1 >= good2 && good1 >= good3 && good1 >= good4)
        {
            R1.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask1.copyTo(_mask);
            return good1;
        }
        else if (good2 >= good1 && good2 >= good3 && good2 >= good4)
        {
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask2.copyTo(_mask);
            return good2;
        }
        else if (good3 >= good1 && good3 >= good2 && good3 >= good4)
        {
            t = -t;
            R1.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask3.copyTo(_mask);
            return good3;
        }
        else
        {
            t = -t;
            R2.copyTo(_R);
            t.copyTo(_t);
            if (_mask.needed()) mask4.copyTo(_mask);
            return good4;
        }
    }

    int recoverPose( InputArray E, InputArray _points1, InputArray _points2, OutputArray _R,
                         OutputArray _t, double focal, Point2d pp, InputOutputArray _mask)
    {
        Mat cameraMatrix = (Mat_<double>(3,3) << focal, 0, pp.x, 0, focal, pp.y, 0, 0, 1);
        return cv::recoverPose(E, _points1, _points2, cameraMatrix, _R, _t, _mask);
    }
}

Vector3d singleTriangle(Vector3d f1, Vector3d f2, const Matrix3d& R21, const Vector3d& t21) {
    Matrix<double, 3, 2> A;
    Vector3d b = -1.0*t21;

    A.col(0) =   R21*f1;
    A.col(1) = -1.0f*f2;

    Matrix2d ATA = A.transpose()*A;
    Vector2d ATb = A.transpose()*b;

    if (ATA.determinant() < 1.0e-5f) {
        return Vector3d(0,0,-1);
    }

    Vector2d x = ATA.ldlt().solve(ATb);

    Vector3d pt3d1 = f1*x(0);
    Vector3d pt3d2 = R21.transpose()*(f2*x(1) - t21);

    return (pt3d1+pt3d2)*0.5;
}

int computeRelativeRT(const vector<Point2f>& pts1, const vector<Point2f>& pts2, Matrix3d& Rcr, Vector3d& tcr) {
    if (pts1.size() <= 15) {
        return false;
    }

    cv::Mat mask;
    cv::Mat E = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 0.3 / FOCAL_LENGTH, 0.99, mask);
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat rot, trans;
    int inlier_cnt = cv::recoverPose(E, pts1, pts2, cameraMatrix, rot, trans, mask);

    cv::cv2eigen(rot, Rcr);
    cv::cv2eigen(trans, tcr);
    
    return inlier_cnt;
}

int trianglesTwoFrame(int id1, int id2, map<int, Feature*>& all_ftr, const vector<FrameStruct*>& all_frames) {
    const FrameStruct& frame_r = *(all_frames[id1]);
    const FrameStruct& frame_c = *(all_frames[id2]);

    const Matrix3d &Rrw = frame_r.Rcw_, &Rcw = frame_c.Rcw_;
    const Vector3d &trw = frame_r.tcw_, &tcw = frame_c.tcw_;

    const Matrix3d& Rcr = Rcw*Rrw.transpose();
    const Vector3d& tcr = tcw-Rcr*trw;

    set<int> covisual;
    const set<int>& set1 = frame_r.feature_ids_;
    const set<int>& set2 = frame_c.feature_ids_;
    set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(), inserter(covisual, covisual.begin()));
    
    int cnt = 0;
    for (int id : covisual) {
        Feature* ftr = all_ftr[id];

        if (ftr->pt3d_.z() > 0) {
            continue;
        }

        Vector3d f1 = ftr->getF(id1).cast<double>();
        Vector3d f2 = ftr->getF(id2).cast<double>();

        // Matrix<double, 3, 4> pose0, pose1;
        // pose0.block<3, 3>(0, 0) = Rrw;
        // pose0.col(3) = trw;

        // pose1.block<3, 3>(0, 0) = Rcw;
        // pose1.col(3) = tcw;

        Vector3d pt3d = singleTriangle(f1, f2, Rcr, tcr);
        // Vector3d pt3d = trianglePoint(pose0, pose1, f1.head<2>(), f2.head<2>());
        if (pt3d.z() < 0) {
            continue;
        }

        cnt ++;
        // ftr->pt3d_ = pt3d;
        ftr->pt3d_ = Rrw.transpose()*(pt3d - trw);
    }
    
    return cnt;
}

int solveRTByPnP(int id1, map<int, Feature*>& all_ftr, set<int>& vis_ftr_id, Matrix3d& Rcr, Vector3d& tcr) {
    
    int cnt = 0;
    vector<Point3f> point3d;
    vector<Point2f> point2d;

    for (int id : vis_ftr_id) {
        Feature* ftr = all_ftr[id];

        if (ftr->pt3d_.z() < 0) {
            continue;
        }

        Vector3f norm_point = ftr->getF(id1);
        point3d.push_back(Point3f(ftr->pt3d_.x(), ftr->pt3d_.y(), ftr->pt3d_.z()));
        point2d.push_back(Point2f(norm_point.x(), norm_point.y()));
    }

    if (point3d.size() < 15) {
        return 0;
    }

    cv::Mat R, r, t;
    
    cv::eigen2cv(Rcr, R);
    cv::Rodrigues(R, r);
    cv::eigen2cv(tcr, t);
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool ret  = cv::solvePnP(point3d, point2d, K, cv::Mat(), r, t, true, 0);

    if (!ret) {
        return 0;
    }

    cv::Rodrigues(r, R);
    cv::cv2eigen(R, Rcr);
    cv::cv2eigen(t, tcr);

    return 1;
}

int globalRefineBA(map<int, Feature*>& all_ftr, vector<FrameStruct*>& frames, int l) {
    int N = frames.size();

    // poses
    vector<pair<double*, double*>> Tcws(N);    
    for (int i = 0; i < N; i++) {
        Quaterniond Qcw(frames[i]->Rcw_);
        Vector3d    tcw(frames[i]->tcw_);

        double* tcws = new double[3];
        tcws[0] = tcw.x();
        tcws[1] = tcw.y();
        tcws[2] = tcw.z();

        double* qcws = new double[4];
        qcws[0] = Qcw.w();
        qcws[1] = Qcw.x();
        qcws[2] = Qcw.y();
        qcws[3] = Qcw.z();

        Tcws[i] = make_pair(tcws, qcws);
    }

    // xyz_point
    unordered_map<int, double*> xyzs;
    for (auto& pir : all_ftr) {
        int ftr_id   = pir.first;
        Feature* ftr = pir.second;
        if (ftr->pt3d_.z() < 0) {
            continue;
        }

        xyzs[ftr_id] = new double[3];
        xyzs[ftr_id][0] = ftr->pt3d_.x();
        xyzs[ftr_id][1] = ftr->pt3d_.y();
        xyzs[ftr_id][2] = ftr->pt3d_.z();
    }


    // build BA graph
    int edge_count = 0;
    ceres::Problem problem;

    for (int i = 0; i < N; i ++) {
        FrameStruct* frame = frames[i];
        
        ceres::LocalParameterization* local_param = new ceres::QuaternionParameterization();
        problem.AddParameterBlock(Tcws[i].second, 4, local_param);
        problem.AddParameterBlock(Tcws[i].first,  3);

        if (i == l) {
            problem.SetParameterBlockConstant(Tcws[i].second);
        }

        if (i == l || i == N-1) {
            problem.SetParameterBlockConstant(Tcws[i].first);
        }

        // scan all feature id
        for (int ftr_id : frame->feature_ids_) {
            Feature* ftr = all_ftr[ftr_id];
            if (ftr->pt3d_.z() < 0) {
                continue;
            }

            if (!xyzs.count(ftr_id)) {
                continue;
            }

            problem.AddParameterBlock(xyzs[ftr_id], 3);

            Vector3f obs = ftr->getF(i);
            ceres::CostFunction* cost = ReprojectionError3D::Create(obs.x(), obs.y());

            problem.AddResidualBlock(cost, NULL, Tcws[i].second, Tcws[i].first, xyzs[ftr_id]);

            edge_count ++;
        }
    }

    cout << "edge count: " << edge_count << endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_solver_time_in_seconds   = 10.0f;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//std::cout << summary.BriefReport() << "\n";
	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03) {
        LOGI("[vision only BA] Converged");
    }
	else {
        LOGI("[vision only BA] Not converged");
		return false;
	}

    // if converged, re-set pose and landmark pose   
    for (int i = 0; i < N; i++) {
        double* tcw = Tcws[i].first;
        double* qcw = Tcws[i].second;

        frames[i]->Rcw_ = Quaterniond(qcw[0], qcw[1], qcw[2], qcw[3]).toRotationMatrix();
        frames[i]->tcw_ = Vector3d(tcw[0], tcw[1], tcw[2]);

        delete(tcw);
        delete(qcw);
    }

    // xyz_point
    for (auto& pir : all_ftr) {
        int ftr_id   = pir.first;
        Feature* ftr = pir.second;
        if (ftr->pt3d_.z() < 0) {
            continue;
        }

        if (!xyzs.count(ftr_id)) {
            continue;
        }

        ftr->pt3d_ = Vector3d(xyzs[ftr_id]);

        delete(xyzs[ftr_id]);
    }

    Tcws.clear();
    xyzs.clear();

    return 1;
}

