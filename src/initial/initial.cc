#include "../../include/initial/initial.h"
#include "../../include/visual/globalsfm.h"
#include "../../include/util/config.h"
#include "../../include/util/utils.h"
#include "../../include/util/log.h"


int relativeRT(map<int, Feature*>& all_ftr, Matrix3d& Rcr, Vector3d& tcr, int window_size) {
    // get a relative RT
    vector<Point2f> ref_pts;
    vector<Point2f> cur_pts;

    vector<Point2f> ref_im_pts;
    vector<Point2f> cur_im_pts;

    // scan from first frame
    int cur_frame_id = window_size;
    int ref_frame_id = 0;
    for (ref_frame_id = 0; ref_frame_id < window_size; ref_frame_id++) {
        
        vector<int> ftr_ids;
        ftr_ids.reserve(100);

        for (auto& pr: all_ftr) {
            int ftr_id = pr.first;
            if (   pr.second->contains(cur_frame_id)
                && pr.second->contains(ref_frame_id)) {
                ftr_ids.push_back(ftr_id);
            }
        }

        if (ftr_ids.empty()) {
            continue;
        }

        ref_pts.clear();
        cur_pts.clear();

        ref_im_pts.clear();
        cur_im_pts.clear();

        double parallax_sum = 0;
        int    parallax_num = 0;
        for (int id : ftr_ids) {
            Vector3d p1 = all_ftr[id]->getF(ref_frame_id);
            Vector3d p2 = all_ftr[id]->getF(cur_frame_id);

            Vector2d ip1 = all_ftr[id]->getUV(ref_frame_id);
            Vector2d ip2 = all_ftr[id]->getUV(cur_frame_id);

            parallax_sum += (p1-p2).norm();
            parallax_num += 1;

            ref_pts.emplace_back(p1.x(), p1.y());
            cur_pts.emplace_back(p2.x(), p2.y());

            ref_im_pts.emplace_back(ip1.x(), ip1.y());
            cur_im_pts.emplace_back(ip2.x(), ip2.y());
        }

        // {   // test match
        //     cv::Mat im(ROW, COL, CV_8UC3, cv::Scalar::all(0));

        //     for (int i = 0; i < ref_im_pts.size(); i++) {
        //         int r = cv::theRNG().uniform(0, 255);
        //         int g = cv::theRNG().uniform(0, 255);
        //         int b = cv::theRNG().uniform(0, 255);

        //         cv::circle(im, ref_im_pts[i], 2, cv::Scalar(r, g, b), 1);
        //         cv::circle(im, cur_im_pts[i], 2, cv::Scalar(r, g, b), 1);

        //         cv::line(im, ref_im_pts[i], cur_im_pts[i], cv::Scalar(r, g, b), 1);

        //         cv::putText(im, to_string(ftr_ids[i]), cur_im_pts[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));
        //     }

        //     cv::imwrite("/home/ubuntu/catkin_ws/src/learn_vio/images/RT.png", im);
            
        // }

        float parallax = FOCAL_LENGTH*parallax_sum/(float)parallax_num;
        LOGD("[relative RT] [%d, %d] parallax is %f|%d", ref_frame_id, cur_frame_id, parallax, parallax_num);
        if (parallax_num < 20 || parallax < 30) {
            continue;
        }

        int inlier = computeRelativeRT(ref_pts, cur_pts, Rcr, tcr);
        if (inlier > 15) {
            LOGD("[relative RT] good inlier: %d, frame id: %d", inlier, ref_frame_id);
            return ref_frame_id;
        }
        LOGD("[relative RT] bad inlier: %d, frame id: %d", inlier, ref_frame_id);
    }

    return -1;
}

int globalSFM(map<int, Feature*>& all_ftr, vector<FrameStruct*>& frames, Matrix3d& Rcl, Vector3d& tcl, int l) {
    int frame_nums = frames.size();

    // triangle all point l and current see
    frames[l]->Rcw_.setIdentity();
    frames[l]->tcw_.setZero();

    frames.back()->Rcw_ = Rcl;
    frames.back()->tcw_ = tcl;

    int temp_cnt = trianglesTwoFrame(l, frame_nums-1, all_ftr, frames);
    LOGD("[global SFM] %d--%d triangled %d landmark", l, frame_nums-1, temp_cnt);

    // from newest to l
    vector<int> fails;

    Matrix3d Rcw = Rcl;
    Vector3d tcw = tcl;
    for (int i = frame_nums-2; i > l; i--) {

        int ret = solveRTByPnP(i, all_ftr, frames[i]->feature_ids_, Rcw, tcw);
        if (ret == 0) {
            fails.push_back(i);
            continue;
        }

        frames[i]->Rcw_ = Rcw;
        frames[i]->tcw_ = tcw;
        // triangle 3D point by two frame
        temp_cnt = trianglesTwoFrame(l, i, all_ftr, frames);
        LOGD("[global SFM] %d--%d triangled %d landmark", l, i, temp_cnt);
    }

    for (int i = l+1; i < frame_nums-1; i++) {
        temp_cnt = trianglesTwoFrame(frame_nums-1, i, all_ftr, frames);
        LOGD("[global SFM] %d--%d triangled %d landmark", frame_nums-1, i, temp_cnt);
    }

    Rcw = frames[l]->Rcw_;
    tcw = frames[l]->tcw_;
    // from lastest to l
    for (int i = l-1; i >= 0; i--) {

        int ret = solveRTByPnP(i, all_ftr, frames[i]->feature_ids_, Rcw, tcw);
        if (ret == 0) {
            fails.push_back(i);
            continue;
        }

        frames[i]->Rcw_ = Rcw;
        frames[i]->tcw_ = tcw;
        // triangle 3D point by two frame
        temp_cnt = trianglesTwoFrame(l, i, all_ftr, frames);
        LOGD("[global SFM] %d--%d triangled %d landmark", l, i, temp_cnt);
    }

    // re initial failed frames
    for (auto iter = fails.begin(); iter != fails.end();) {
        Rcw.setIdentity();
        tcw.setZero(); 
        
        int i = *iter;
        int ret = solveRTByPnP(i, all_ftr, frames[i]->feature_ids_, Rcw, tcw);
        
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
    int new_triangle_point   = 0;
    int vaild_point_cnt      = 0;
    int unvaild_point_cnt    = 0;
    int untriangle_point_cnt = 0;
    FILE* file = NULL;

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

        Vector3d f1 = ftr->vis_fs_[0].cast<double>();
        Vector3d f2 = ftr->vis_fs_.back().cast<double>();

        Matrix3d Rc1w = frames[ftr->ref_frame_id_]->Rcw_;
        Vector3d tc1w = frames[ftr->ref_frame_id_]->tcw_;

        Matrix3d Rc2w = frames[ftr->ref_frame_id_+ftr->size()-1]->Rcw_;
        Vector3d tc2w = frames[ftr->ref_frame_id_+ftr->size()-1]->tcw_;

        Matrix3d R21  = Rc2w*Rc1w.transpose();
        Vector3d t21  = tc2w - R21*tc1w;

        Vector3d pt3d = singleTriangle(f1, f2, R21, t21);
        // Vector3d pt3d = trianglePoint(pose0, pose1, f1.head<2>(), f2.head<2>());

        if (pt3d.z() < 0) {
            untriangle_point_cnt++;
            continue;
        } 

        Vector3d pt3d_w = Rc1w.transpose()*(pt3d - tc1w);

        ftr->pt3d_ = pt3d_w;
        new_triangle_point++;
    }

    LOGD("[global SFM] valid points:        %d|%d=%.2f", vaild_point_cnt,      (int)all_ftr.size(), vaild_point_cnt/(float)all_ftr.size());
    LOGD("[global SFM] new triangle points: %d|%d=%.2f", new_triangle_point,   (int)all_ftr.size(), new_triangle_point/(float)all_ftr.size());
    LOGD("[global SFM] unvalid points:      %d|%d=%.2f", unvaild_point_cnt,    (int)all_ftr.size(), unvaild_point_cnt/(float)all_ftr.size());
    LOGD("[global SFM] untriangle points:   %d|%d=%.2f", untriangle_point_cnt, (int)all_ftr.size(), untriangle_point_cnt/(float)all_ftr.size());

    LOGD("[global SFM] Before BA");

    // TODO: full BA
    if (!globalRefineBA(all_ftr, frames, l)) {
        LOGW("[global SFM] BA failed");
        return 0;
    }

    LOGD("[global SFM] After BA");

    // move the relative RT to first frame
    Matrix3d Rwl;
    Vector3d twl;

    int i = 0;
    // file = fopen("./poses.txt", "w");
    for (FrameStruct* frame_i : frames) {
        if (i == 0) {
            Rwl = frame_i->Rcw_;
            twl = frame_i->tcw_;

            frame_i->Rcw_.setIdentity();
            frame_i->tcw_.setZero();
        } 
        else {
            Matrix3d Ril = frame_i->Rcw_;
            Vector3d til = frame_i->tcw_;

            Matrix3d Rwi = Rwl*Ril.transpose();
            Vector3d twi = twl-Rwi*til;

            frame_i->Rcw_ = Rwi.transpose();
            frame_i->tcw_ = Rwi.transpose()*twi*-1;
        }

        // camera pose
        double timestamp = frame_i->t_;

        Quaterniond Qwc = Quaterniond(frame_i->Rcw_.transpose());
        Vector3d    twc = Qwc*frames[i]->tcw_*-1.0;

        // fprintf(file, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", 
        //         timestamp, Qwc.w(), Qwc.x(), Qwc.y(), Qwc.z(), twc.x(), twc.y(), twc.z());

        i++;
    }
    // fclose(file);

    // move the point to first frame
    // file = fopen("./landmark.txt", "w");
    for (auto& pir : all_ftr) {
        Feature* ftr = pir.second;
        if (ftr->pt3d_.z() < 0) {
            continue;
        }

        ftr->pt3d_ = Rwl*ftr->pt3d_ + twl;
        // fprintf(file, "%d\t%lf\t%lf\t%lf\n", pir.first, ftr->pt3d_.x(), ftr->pt3d_.y(), ftr->pt3d_.z());
    }
    // fclose(file);

    return 1;
}

int visualOnlyInit(map<int, Feature*>& all_ftr, vector<FrameStruct*>& all_frames) {
    int N = all_frames.size()-1;
    
    Matrix3d Rcr;
    Vector3d tcr;
    int l = relativeRT(all_ftr, Rcr, tcr, N);
    if (l < 0) {
        LOGW("[vis only initial] can not get good RT, failed!!");
        return 0;
    }

    LOGD("[vis only initial] get a good RT %d-->%d", l, N);
    
    // global SFM
    int ret = globalSFM(all_ftr, all_frames, Rcr, tcr, l);
    if (ret == 0) {
        LOGW("[vis only initial] global sfm failed!!!");
        return 0;
    }

    LOGD("[vis only initial] global sfm success!!!");
    return 1;
}

Vector3d computeGyroBias(vector<FrameStruct*>& frames) {
    Matrix3d A;
    Vector3d b;

    A.setZero();
    b.setZero();
    for (int i = 1; i < frames.size(); i++) {
        // adjacent two frame
        FrameStruct* frame_i = frames[i-1];
        FrameStruct* frame_j = frames[i];

        PreIntegrate* preinte = frames[i]->preintegrate_;
        
        // 
        Quaterniond Qij_v = Quaterniond(frame_i->Rcw_*frame_j->Rcw_.transpose());
        Quaterniond Qij_i = preinte->delta_q_.cast<double>();

        Matrix3d Jacobian = preinte->Jacobian_.block<3, 3>(J_OR, J_OBW).cast<double>();
        Vector3d Error    = 2*(Qij_v.inverse()*Qij_i).vec();

        A.noalias() += Jacobian.transpose()*Jacobian;
        b.noalias() += Jacobian.transpose()*Error;
    }

    Vector3d delta_bg = A.ldlt().solve(-b);

    return delta_bg;
}

Matrix<double, 3, 2> tangentBasic(Vector3d &g0) {
    Vector3d b1, b2;
    if (g0 != Vector3d(1, 0, 0)) {
        b1 = g0.cross(Vector3d(1, 0, 0));
        b1.normalize();
    }
    else {
        b1 = g0.cross(Vector3d(0, 0, 1));
        b1.normalize();
    }

    b2 = b1.cross(g0);
    b2.normalize();

    Matrix<double, 3, 2> ret;
    ret.col(0) = b1;
    ret.col(1) = b2;

    return ret;
}

int refineGravity(vector<FrameStruct*>& frames, Matrix3d qwb[], Vector3d pwc[], Matrix3d &Rbc, Vector3d &tbc,
                  VectorXd &x, Vector3d &g0) {
    const int N = frames.size();

    MatrixXd ATA(3*N+2+1, 3*N+2+1);
    VectorXd ATb(3*N+2+1);

    Vector3d g_0 = g0.normalized()*9.81;

    for (int j = 0; j < 4; j++) {
        Matrix<double, 3, 2> basic = tangentBasic(g_0);


        ATA.setZero();
        ATb.setZero();
        int i = 0;
        for (i = 0; i < N-1; i++) {
            FrameStruct* frame_i = frames[i-0];
            FrameStruct* frame_j = frames[i+1];

            // bk<-bk+1
            Vector3d Delta_p = frame_j->preintegrate_->delta_p_.cast<double>();
            Vector3d Delta_v = frame_j->preintegrate_->delta_v_.cast<double>();

            double sum_t = frame_j->preintegrate_->sum_dt_;

            MatrixXd A(6, 9);
            VectorXd b(6);

            A.setZero();
            b.setZero();

            // vbk vbk+1
            A.block<3, 3>(0, 0) = -sum_t*Matrix3d::Identity();
            A.block<3, 2>(0, 6) = 0.5*sum_t*sum_t*qwb[i].transpose()*basic;
            A.block<3, 1>(0, 8) = qwb[i].transpose()*(pwc[i+1]-pwc[i+0])/100.0;
            b.segment<3>(0)     = Delta_p - tbc + qwb[i].transpose()*qwb[i+1]*tbc - 0.5*sum_t*sum_t*qwb[i].transpose()*g_0;

            A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            A.block<3, 3>(3, 3) = qwb[i].transpose()*qwb[i+1];
            A.block<3, 2>(3, 6) = qwb[i].transpose()*basic*sum_t;
            b.segment<3>(3)     = Delta_v - sum_t*qwb[i].transpose()*g_0;

            MatrixXd tmp_ATA = A.transpose()*A; // 9x9
            VectorXd tmp_ATb = A.transpose()*b; // 9x1

            ATA.block<6, 6>(i*3, i*3) += tmp_ATA.topLeftCorner<6, 6>();
            ATb.segment<6>(i*3)       += tmp_ATb.head<6>();

            ATA.block<6, 3>(i*3, N*3) += tmp_ATA.topRightCorner<6, 3>();
            ATA.block<3, 6>(N*3, i*3) += tmp_ATA.bottomLeftCorner<3, 6>();

            ATA.block<3, 3>(N*3, N*3) += tmp_ATA.bottomRightCorner<3, 3>();
            ATb.segment<3>(N*3)       += tmp_ATb.tail<3>();
        }

        ATA *= 1000;
        ATb *= 1000;

        x = ATA.ldlt().solve(ATb);
        
        Vector2d w12  = x.segment<2>(N*3);
        g_0 += basic*w12;

        g_0.normalize();
        g_0 *= 9.81;
    }

    g0 = g_0;
    return 1;   
}

int VisualInertialSolve(vector<FrameStruct*>& frames,  Matrix3d Rbc, Vector3d tbc, 
                        VectorXd& x, Vector3d& g_c0) {
    const int N = frames.size();

    Matrix3d qwb[N];
    Vector3d pwc[N];

    int i = 0;
    for (FrameStruct* frame: frames) {
        qwb[i] = frame->Rcw_.transpose()*Rbc.transpose();
        pwc[i] = frame->Rcw_.transpose()*frame->tcw_*-1.0;
        i++;
    }

    MatrixXd ATA(3*N+3+1, 3*N+3+1);
    VectorXd ATb(3*N+3+1);
    
    ATA.setZero();
    ATb.setZero();
    for (i = 0; i < N-1; i++) {
        FrameStruct* frame_i = frames[i-0];
        FrameStruct* frame_j = frames[i+1];

        // bk<-bk+1
        Vector3d Delta_p = frame_j->preintegrate_->delta_p_.cast<double>();
        Vector3d Delta_v = frame_j->preintegrate_->delta_v_.cast<double>();

        double sum_t = frame_j->preintegrate_->sum_dt_;

        MatrixXd A(6, 10);
        VectorXd b(6);

        A.setZero();
        b.setZero();

        // vbk vbk+1
        A.block<3, 3>(0, 0) = -sum_t*Matrix3d::Identity();
        A.block<3, 3>(0, 6) = 0.5*sum_t*sum_t*qwb[i].transpose();
        A.block<3, 1>(0, 9) = qwb[i].transpose()*(pwc[i+1]-pwc[i+0])/100.0;
        b.segment<3>(0)     = Delta_p - tbc + qwb[i].transpose()*qwb[i+1]*tbc;

        A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        A.block<3, 3>(3, 3) = qwb[i].transpose()*qwb[i+1];
        A.block<3, 3>(3, 6) = qwb[i].transpose()*sum_t;
        b.segment<3>(3)     = Delta_v;

        MatrixXd tmp_ATA = A.transpose()*A; // 10x10
        VectorXd tmp_ATb = A.transpose()*b; // 10x1

        ATA.block<6, 6>(i*3, i*3) += tmp_ATA.topLeftCorner<6, 6>();
        ATb.segment<6>(i*3)       += tmp_ATb.head<6>();

        ATA.block<6, 4>(i*3, N*3) += tmp_ATA.topRightCorner<6, 4>();
        ATA.block<4, 6>(N*3, i*3) += tmp_ATA.bottomLeftCorner<4, 6>();

        ATA.block<4, 4>(N*3, N*3) += tmp_ATA.bottomRightCorner<4, 4>();
        ATb.segment<4>(N*3)       += tmp_ATb.tail<4>();
    }

    ATA *= 1000;
    ATb *= 1000;

    VectorXd X;
    X = ATA.ldlt().solve(ATb);

    g_c0     = X.segment<3>(N*3);
    double s = X.tail<1>()[0]/100;

    LOGW("[Align] before Gravity refine: %.4lf, %.4lf, %.4lf, s: %.4lf",  g_c0(0), g_c0(1), g_c0(2), s);
    
    if (abs(g_c0.norm()-9.81) > 1.0) {
        LOGW("[Align] Gravity is bad");
        return 0;
    }

    // refine gravity
    refineGravity(frames, qwb, pwc, Rbc, tbc, x, g_c0);
    
    // reset the scale
    s = x.tail<1>()[0]/100;
    x.tail<1>()[0] = s;
    LOGW("[Align] after Gravity refine: %.4lf, %.4lf, %.4lf, s: %.4lf",  g_c0(0), g_c0(1), g_c0(2), s);
    
    if (abs(g_c0.norm()-9.81) > 1.0) {
        LOGW("[Align] Gravity is bad");
        return 0;
    }

    LOGW("[Align] Gravity refine is success!!!");
    return 1;
}

int visualInertialAlign(vector<FrameStruct*>& frames, Matrix3d Rbc, Vector3d tbc, Vector3d &g_c0, double &s) {
    // 1. compute gyro bias
    Vector3d bias_g = computeGyroBias(frames);

    LOGW("[Align] Gyro bias: %.4lf, %.4lf, %.4lf", bias_g.x(), bias_g.y(), bias_g.z());

    for (FrameStruct *frame: frames) {
        if (frame->preintegrate_ == nullptr) {
            frame->bias_g_ = bias_g;
            continue;
        }
        frame->preintegrate_->gyro_bias_ = bias_g;
        frame->preintegrate_->reintegrate(Vector3d(0, 0, 0), bias_g);
        frame->bias_g_ = bias_g;
    }

    // 2. visual inertial alignment 
    VectorXd x; x.setZero();
    if (0 == VisualInertialSolve(frames, Rbc, tbc, x, g_c0)) {
        return 0;
    }

    s = x.tail<1>()[0];

    int i = 0;
    for (FrameStruct *frame: frames) {
        frame->Vbk_ = x.segment<3>(i*3);
        i++;
    }

    return 1;
}

