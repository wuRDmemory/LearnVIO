#include "../../include/visual/globalsfm.h"
#include "../../include/util/utils.h"
#include "../../include/util/config.h"

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

Vector3f singleTriangle(Vector3f& f1, Vector3f& f2, const Matrix3f& R21, const Vector3f& t21) {
    Matrix<float, 3, 2> A;
    Vector3f b = t21;

    A.col(0) =   R21*f1;
    A.col(1) = -1.0f*f2;

    Matrix2f ATA = A.transpose()*A;
    Vector2f ATb = A.transpose()*b;

    if (ATA.determinant() < 1.0e-5f) {
        return Vector3f(0,0,-1);
    }

    Vector2f x = -ATA.inverse()*ATb;

    Vector3f pt3d1 = f1*x(0);
    Vector3f pt3d2 = R21.transpose()*(f2*x(1) - t21);

    return (pt3d1+pt3d2)/2;
}

int computeRelativeRT(const vector<Point2f>& pts1, const vector<Point2f>& pts2, Matrix3f& Rcr, Vector3f& tcr) {
    if (pts1.size() <= 15) {
        return false;
    }

    cv::Mat mask;
    cv::Mat E = cv::findFundamentalMat(pts1, pts2, cv::FM_RANSAC, 0.3 / FOCAL_LENGTH, 0.99, mask);
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat rot, trans;
    int inlier_cnt = cv::recoverPose(E, pts1, pts2, cameraMatrix, rot, trans, mask);

    for (int i = 0; i < 3; i++) {   
        tcr(i) = trans.at<double>(i, 0);
        for (int j = 0; j < 3; j++)
            Rcr(i, j) = rot.at<double>(i, j);
    }
    
    return inlier_cnt;
}


int trianglesTwoFrame(int id1, int id2, map<int, Feature*>& all_ftr, const vector<FrameStruct>& all_frames) {
    const FrameStruct& frame_r = all_frames[id1];
    const FrameStruct& frame_c = all_frames[id2];

    const Matrix3f& Rcr = frame_c.Rcw_;
    const Vector3f& tcr = frame_c.tcw_;

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

        Vector3f f1 = ftr->getF(id1);
        Vector3f f2 = ftr->getF(id2);

        Vector3f pt3d = singleTriangle(f1, f2, Rcr, tcr);
        if (pt3d.z() < 0) {
            continue;
        }

        cnt ++;
        ftr->pt3d_ = pt3d;
    }
    
    return cnt;
}

int solveRTByPnP(int id1, map<int, Feature*>& all_ftr, set<int>& vis_ftr_id, Matrix3f& Rcr, Vector3f& tcr) {
    
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
    cv::Mat K = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    bool ret  = cv::solvePnP(point3d, point2d, K, cv::Mat(), r, t, true);

    if (!ret) {
        return 0;
    }

    cv::Rodrigues(r, R);
    Matrix3d Rcrd;
    Vector3d tcrd;

    cv::cv2eigen(R, Rcrd);
    cv::cv2eigen(t, tcrd);

    Rcr = Rcrd.cast<float>();
    tcr = tcrd.cast<float>();

    return 1;
}

int globalSFM(map<int, Feature*>& all_ftr, vector<FrameStruct>& frames, Matrix3f& Rcl, Vector3f& tcl, int l) {
    int frame_nums = frames.size();

    // triangle all point l and current see
    frames[0].Rcw_.setIdentity();
    frames[1].tcw_.setZero();

    frames.back().Rcw_ = Rcl;
    frames.back().tcw_ = tcl;

    trianglesTwoFrame(l, frame_nums-1, all_ftr, frames);

    // from newest to l
    vector<int> fails;

    Matrix3f Rcw = Rcl;
    Vector3f tcw = tcl;
    for (int i = frame_nums-2; i > l; i--) {

        int ret = solveRTByPnP(i, all_ftr, frames[i].feature_ids_, Rcw, tcw);
        if (ret == 0) {
            fails.push_back(i);
            continue;
        }

        frames[i].Rcw_ = Rcw;
        frames[i].tcw_ = tcw;
        // triangle 3D point by two frame
        trianglesTwoFrame(l, i, all_ftr, frames);
    }

    Rcw.setIdentity();
    tcw.setZero();
    // from lastest to l
    for (int i = 0; i < l; i++) {

        int ret = solveRTByPnP(i, all_ftr, frames[i].feature_ids_, Rcw, tcw);
        if (ret == 0) {
            fails.push_back(i);
            continue;
        }

        frames[i].Rcw_ = Rcw;
        frames[i].tcw_ = tcw;
        // triangle 3D point by two frame
        trianglesTwoFrame(l, i, all_ftr, frames);
    }

    // re initial failed frames
    for (auto iter = fails.begin(); iter != fails.end();) {
        Rcw.setIdentity();
        tcw.setZero(); 
        
        int i = *iter;
        int ret = solveRTByPnP(i, all_ftr, frames[i].feature_ids_, Rcw, tcw);
        
        if (ret == 0) {
            iter++;
            continue;
        }

        iter = fails.erase(iter);
    }

    if (!fails.empty()) {
        LOGW("[global sfm] some frame can not be locate");
        return 0;
    }

    // triangle all features
    int vaild_point_cnt      = 0;
    int unvaild_point_cnt    = 0;
    int untriangle_point_cnt = 0;
    for (auto& pir : all_ftr) {
        Feature* ftr = pir.second;
        
        if (ftr->pt3d_.z() > 0) {
            vaild_point_cnt ++;
            continue;
        }

        if (ftr->size() < 2) {
            unvaild_point_cnt ++;
            continue;
        }

        Vector3f f1 = ftr->vis_fs_[0];
        Vector3f f2 = ftr->vis_fs_.back();

        Matrix3f Rc1w = frames[ftr->ref_frame_id_].Rcw_;
        Vector3f tc1w = frames[ftr->ref_frame_id_].tcw_;

        Matrix3f Rc2w = frames[ftr->ref_frame_id_+ftr->size()-1].Rcw_;
        Vector3f tc2w = frames[ftr->ref_frame_id_+ftr->size()-1].tcw_;

        Matrix3f R21  = Rc2w*Rc1w.transpose();
        Vector3f t21  = tc2w - R21*tc1w;

        Vector3f pt3d = singleTriangle(f1, f2, R21, t21);

        if (pt3d.z() < 0) {
            untriangle_point_cnt++;
            continue;
        } 

        Vector3f pt3d_w = Rc1w.transpose()*(pt3d - tc1w);

        ftr->pt3d_ = pt3d_w;
        vaild_point_cnt++;
    }

    LOGI("[global SFM] valid points:      %d|%d=%.2f", vaild_point_cnt,      (int)all_ftr.size(), vaild_point_cnt/(float)all_ftr.size());
    LOGI("[global SFM] unvalid points:    %d|%d=%.2f", unvaild_point_cnt,    (int)all_ftr.size(), unvaild_point_cnt/(float)all_ftr.size());
    LOGI("[global SFM] untriangle points: %d|%d=%.2f", untriangle_point_cnt, (int)all_ftr.size(), untriangle_point_cnt/(float)all_ftr.size());

    for (int i = 0; i < frames.size(); i++) {
        cout << i << ": " << frames[i].tcw_.transpose() << endl;
    }

    return 1;
}
