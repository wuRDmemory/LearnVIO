#include "../include/estimator.h"
#include "../include/util/utils.h"
#include "../include/util/log.h"
#include "../include/util/config.h"
#include "../include/initial/initial.h"
#include "../include/visual/globalsfm.h"
#include "../include/visual/project_factor.h"
#include "../include/inertial/inertial_factor.h"

Estimator::Estimator() {
    temp_preintegrate_= nullptr;
    clearState();
}

Estimator::~Estimator() {
    ;
}

void Estimator::clearState() {
    frame_count_ = 0;
    first_imu_   = true;
    initial_     = false;

    if (temp_preintegrate_ != nullptr) {
        delete temp_preintegrate_;
    }
    temp_preintegrate_ = NULL;

    preintegrates_.resize(FEN_WINDOW_SIZE+1);

    RS_.resize(FEN_WINDOW_SIZE+1);
    VS_.resize(FEN_WINDOW_SIZE+1);
    PS_.resize(FEN_WINDOW_SIZE+1);

    BAS_.resize(FEN_WINDOW_SIZE+1);
    BGS_.resize(FEN_WINDOW_SIZE+1);

    for (int i = 0; i <= FEN_WINDOW_SIZE; i++) {
        if (preintegrates_[i] != nullptr) {
            delete preintegrates_[i];
        }

        preintegrates_[i] = NULL;

        RS_[i].setIdentity();
        VS_[i].setZero();
        PS_[i].setZero();

        BAS_[i].setZero();
        BGS_[i].setZero();
    }

    g_.setZero();

    feature_manager_.clear();
}

void Estimator::processImu(double dt, Vector3f accl, Vector3f gyro) {
    
    // LOGD("[estimate] FEN_WINDOW_SIZE: %d, frame id %d", FEN_WINDOW_SIZE, frame_count_); 
    if (preintegrates_[frame_count_] == NULL) {
        preintegrates_[frame_count_] = new PreIntegrate(accl_0_, gyro_0_, BAS_[frame_count_], BGS_[frame_count_]);
    } 

    if (frame_count_ != 0) {
        // first frame have no delta
        // so do not integrate
        preintegrates_[frame_count_]->push_back(dt, accl, gyro);
        temp_preintegrate_->push_back(dt, accl, gyro);

        // the prior R V P
        Vector3f gyro_mid;
        Vector3f accl_b0, accl_b1, accl_mid;
        
        gyro_mid = (gyro + gyro_0_)*dt/2;
        accl_b0  = RS_[frame_count_]*(accl_0_-BAS_[frame_count_]);
        
        RS_[frame_count_] *= vec2quat<float>(gyro_mid);
        accl_b1  = RS_[frame_count_]*(accl   -BAS_[frame_count_]);

        accl_mid = (accl_b0 + accl_b1)/2 - Gw;
        PS_[frame_count_] += VS_[frame_count_]*dt + 0.5*accl_mid*dt*dt;
        VS_[frame_count_] += accl_mid*dt;
    }

    accl_0_ = accl;
    gyro_0_ = gyro;
}

void Estimator::processImage(double timestamp, Image_Type& image) {
    LOGI("[estimate2] ========   new image come!!!  ==========");
    bool margin_old = feature_manager_.addNewFeatures(image, frame_count_);
    if (margin_old) {
        LOGI("[estimate] Margin old state, build new key frame");
    } 
    else {
        LOGI("[estimate] Margin new state");
    }

    LOGI("[estimate] Solving %d frame", frame_count_);
    LOGI("[estimate] Feature number : %d", feature_manager_.size());
    timestamp_.push_back(timestamp);

    FrameStruct this_frame(image, timestamp);
    this_frame.preintegrate_ = temp_preintegrate_;
    temp_preintegrate_       = new PreIntegrate(accl_0_, gyro_0_, BAS_[frame_count_], BGS_[frame_count_]);
    all_frames_.insert(make_pair(timestamp, this_frame));

    if (!initial_) {
        if (frame_count_ >= FEN_WINDOW_SIZE) {
            // enough frame
            // initialize
            bool success = structInitial();
            if (!success) {
                // initial success 
                initial_ = true;
                solveOdometry();
                slideWindow(margin_old);
                // TODO: remove the unvisual feature in feature manager

            } 
            else {
                slideWindow(margin_old);
            }
        }
        else {
            frame_count_++;
        }
    }
    else {
        // initial has done

    }
}

bool Estimator::solveOdometry() {
    // solve the current frame's pose
    if (frame_count_ < FEN_WINDOW_SIZE) {
        return false;
    }

    if (!initial_) {
        return false;
    }

    Matrix3f Rwc;
    Vector3f twc;
    Matrix3f Rbc = Rics[0].cast<float>();
    Vector3f tbc = tics[0].cast<float>();
    Matrix3f Rcw[FEN_WINDOW_SIZE+1];
    Vector3f tcw[FEN_WINDOW_SIZE+1];
    for (int i = 0; i <= FEN_WINDOW_SIZE; i++) {
        Rwc = RS_[i].toRotationMatrix()*Rbc; // w_R_c = w_R_b*b_R_c
        twc = PS_[i] + RS_[i]*tbc;           // w_t_c = w_t_bk + w_R_bk*bk_t_ck

        Rcw[i] = Rwc.transpose();
        tcw[i] = Rwc.transpose()*twc*-1;
    }

    int new_ftr_cnt = feature_manager_.trianglesNew(Rcw, tcw);
    LOGD(">>> [new ftr] %d", new_ftr_cnt);

    solveOptimize();

    return true;
}

bool Estimator::solveNewFrame(map<int, Feature*> &all_features, Matrix3f Rwb, Vector3f twb) {
    vector<cv::Point3f> point3d;
    vector<cv::Point2f> point2d;

    Matrix3f Rbc = Rics[0].cast<float>();
    Vector3f tbc = tics[0].cast<float>();

    for (auto &id_ftr : all_features) {
        int       id = id_ftr.first;
        Feature* ftr = id_ftr.second;

        if (!ftr->contains(FEN_WINDOW_SIZE)) {
            continue;
        }

        int ref_id = ftr->ref_frame_id_;
        Matrix3f Rwbr = RS_[ref_id].toRotationMatrix();
        Vector3f twbr = PS_[ref_id];

        Matrix3f Rwcr;
        Vector3f twcr;
        cvtPoseFromBodyToCamera(Rwbr, twbr, Rbc, tbc, Rwcr, twcr);

        Vector3f Pw   = Rwcr*(ftr->vis_fs_[0]/ftr->inv_d_) + twcr;
        Vector3f pc   = ftr->getF(FEN_WINDOW_SIZE);
        assert(pc == ftr->vis_fs_.back());

        point3d.emplace_back(Pw.x(), Pw.y(), Pw.z());
        point2d.emplace_back(pc.x(), pc.y());
    }

    Matrix3f Rwc;
    Vector3f twc;
    cvtPoseFromBodyToCamera(Rwb, twb, Rbc, tbc, Rwc, twc);

    Matrix3d Rcw = Rwc.transpose().cast<double>();
    Vector3d tcw = twc.cast<double>()*Rcw*-1;

    cv::Mat cvRcw, rvec, tvec;
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);

    cv::eigen2cv(Rcw,  cvRcw);
    cv::eigen2cv(tcw,  tvec);
    cv::Rodrigues(cvRcw, rvec);

    cv::solvePnP(point3d, point2d, K, cv::Mat(), rvec, tvec, true, cv::SOLVEPNP_EPNP);

    cv::Rodrigues(rvec, cvRcw);
    cv::cv2eigen(cvRcw, Rcw);
    cv::cv2eigen(tvec,  tcw);

    Rwc = Rcw.transpose().cast<float>();
    twc = (Rcw.transpose()*tcw*-1).cast<float>();

    cvtPoseFromCameraToBody(Rwc, twc, Rbc, tbc, Rwb, twb);

    return true;
}

bool Estimator::structInitial() {

    {   // check imu 
        map<double, FrameStruct>::iterator frame_it;
        Vector3f sum_g;
        for (frame_it = all_frames_.begin(), frame_it++; frame_it != all_frames_.end(); frame_it++) {
            double dt = frame_it->second.preintegrate_->sum_dt_;
            Vector3f tmp_g = frame_it->second.preintegrate_->delta_v_/dt;
            sum_g += tmp_g;
        }
        
        Vector3f aver_g;
        aver_g = sum_g * 1.0 / ((int)all_frames_.size() - 1);
        double var = 0;
        for (frame_it = all_frames_.begin(), frame_it++; frame_it != all_frames_.end(); frame_it++) {
            double dt = frame_it->second.preintegrate_->sum_dt_;
            Vector3f tmp_g = frame_it->second.preintegrate_->delta_v_ / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        }

        var = sqrt(var / ((int)all_frames_.size() - 1));
        if(var < 0.25) {
            // maybe the body coordinate is static
            LOGI("IMU excitation not enouth!");
            //return false;
        }
    }
    
    const int N = FEN_WINDOW_SIZE+1;
    vector<FrameStruct*> all_frames;
    all_frames.reserve(N);
    for (auto& pir : all_frames_) {
        all_frames.push_back(&pir.second);
    }

    if (0 == visualOnlyInit(feature_manager_.all_ftr_, all_frames)) {
        LOGW("[struct initial] struct initial failed!!!");
        return false;
    }

    // visual inertial alignment
    Vector3d g_c0;
    double   s;
    if (0 == visualInertialAlign(all_frames, Rics[0], tics[0], g_c0, s)) {
        LOGW("[struct initial] visual inertial alignment failed!!!");
        return false;
    }

    // alignment is success
    {   // build Qc0b, vc0b, pc0b
        Matrix3d qc0b[N];
        Vector3d vc0b[N];
        Vector3d pc0b[N];

        int i = 0;
        for (FrameStruct *frame: all_frames) {
            const Matrix3d &Rckcw = frame->Rcw_;
            const Vector3d &tckcw = frame->tcw_;

            qc0b[i] = Rckcw.transpose()*Rics[0].transpose();           // c0_Rck*ck_R_bk
            pc0b[i] = Rckcw.transpose()*tckcw*-s - qc0b[i]*tics[0];    // c0_p_ck - c0_R_bk*bk_p_ck
            vc0b[i] = qc0b[i]*frame->Vbk_;                             // c0_R_bk*bk_v_bk
            i++;
        }

        // temporary set camera0 coordination as world
        for (i = N-1; i >= 0; i--) {
            RS_[i] = Quaterniond(qc0b[i]).cast<float>();
            PS_[i] = (pc0b[i]-pc0b[0]).cast<float>();
            VS_[i] = (vc0b[i]).cast<float>();

            BAS_[i] = all_frames[i]->bias_a_.cast<float>();
            BGS_[i] = all_frames[i]->bias_g_.cast<float>();

            preintegrates_[i]->reintegrate(Vector3f(0, 0, 0), BGS_[i]);
        }
    }

    {   // re-triangle all features
        Matrix3f qcc0[N];
        Vector3f tcc0[N];
        Matrix3f Rc0c;
        Vector3f tc0c;

        Matrix3f Ric = Rics[0].cast<float>();
        Vector3f tic = tics[0].cast<float>();

        int i = 0;
        for (i = 0; i < N; i++) {
            Rc0c = RS_[i].toRotationMatrix()*Ric;      // c0_R_c = c0_R_bk*bk_R_ck
            tc0c = PS_[i] + RS_[i]*tic;                // c0_P_c = c0_P_bk + c0_R_bk*bk_P_ck
            
            qcc0[i] = Rc0c.transpose();
            tcc0[i] = Rc0c.transpose()*tc0c*-1;
        }

        feature_manager_.trianglesInitial(qcc0, tcc0);
    }

    // rotate the c0 to world coordinate
    Matrix3f Rw1c0 = gravity2Rnb<float>(g_c0.cast<float>());
    float    yaw   = Rnb2ypr<float>(Rw1c0).x();
    Matrix3f Rww1  = ypr2Rnb<float>(Vector3f{-yaw, 0, 0});

    Matrix3f Rwc0 = Rww1*Rw1c0;
    Gw = Rwc0*g_c0.cast<float>();

    // change world frame from c0 to world
    for (int i = 0; i < N; i++) {
        RS_[i] = Rwc0*RS_[i];
        PS_[i] = Rwc0*PS_[i];
        VS_[i] = Rwc0*VS_[i];
    }

    FILE* file;
    file = fopen("./poses.txt", "w");
    for (int i = 0; i < N; i++) {
        fprintf(file, "%lf\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", 
                    timestamp_[i], RS_[i].x(), RS_[i].y(), RS_[i].z(), RS_[i].w(), 
                                   PS_[i].x(), PS_[i].y(), PS_[i].z());
    }
    fclose(file);

    return true;
}

bool Estimator::slideWindow(bool margin_old) {
    assert(frame_count_ == FEN_WINDOW_SIZE);
    if (margin_old) {
        double t0 = timestamp_[0];
        // marg oldest frame
        for (int i = 0; i < FEN_WINDOW_SIZE; i++) {
            swap(timestamp_[i], timestamp_[i+1]);

            swap(preintegrates_[i], preintegrates_[i+1]);
            
            swap(RS_[ i], RS_[ i+1]);
            swap(VS_[ i], VS_[ i+1]);
            swap(PS_[ i], PS_[ i+1]);
            swap(BAS_[i], BAS_[i+1]);
            swap(BGS_[i], BGS_[i+1]);
        }
        
        RS_[ FEN_WINDOW_SIZE] = RS_[ FEN_WINDOW_SIZE-1];
        VS_[ FEN_WINDOW_SIZE] = VS_[ FEN_WINDOW_SIZE-1];
        PS_[ FEN_WINDOW_SIZE] = PS_[ FEN_WINDOW_SIZE-1];
        BAS_[FEN_WINDOW_SIZE] = BAS_[FEN_WINDOW_SIZE-1];
        BGS_[FEN_WINDOW_SIZE] = BGS_[FEN_WINDOW_SIZE-1];
        timestamp_.pop_back();

        // build new 
        if (preintegrates_[FEN_WINDOW_SIZE]) {
            delete preintegrates_[FEN_WINDOW_SIZE];
            preintegrates_[FEN_WINDOW_SIZE] = nullptr;
        }

        if (!initial_) {
            // remove oldest frame
            map<double, FrameStruct>::iterator iter;
            iter = all_frames_.find(t0);
            assert(iter != all_frames_.end());

            delete(iter->second.preintegrate_);
            iter->second.preintegrate_ = nullptr;

            for (auto it = all_frames_.begin(); it != iter; it++) {
                if (it->second.preintegrate_) {
                    delete(it->second.preintegrate_);
                }
                it->second.preintegrate_ = nullptr;
            }

            all_frames_.erase(all_frames_.begin(), iter);
            all_frames_.erase(t0);
        }   

        slideOldFrame();
    }
    else {
        // marg second newest frame
        auto &accls = preintegrates_[FEN_WINDOW_SIZE]->accl_buf_;
        auto &gyros = preintegrates_[FEN_WINDOW_SIZE]->gyro_buf_;
        auto &dts   = preintegrates_[FEN_WINDOW_SIZE]->dt_buf_;
        for (int i = 1; i < dts.size(); i++) {
            preintegrates_[FEN_WINDOW_SIZE-1]->push_back(dts[i], accls[i], gyros[i]);
        }
        
        RS_[ FEN_WINDOW_SIZE-1] = RS_[ FEN_WINDOW_SIZE];
        VS_[ FEN_WINDOW_SIZE-1] = VS_[ FEN_WINDOW_SIZE];
        PS_[ FEN_WINDOW_SIZE-1] = PS_[ FEN_WINDOW_SIZE];
        BAS_[FEN_WINDOW_SIZE-1] = BAS_[FEN_WINDOW_SIZE];
        BGS_[FEN_WINDOW_SIZE-1] = BGS_[FEN_WINDOW_SIZE];
        timestamp_[FEN_WINDOW_SIZE-1] = timestamp_[FEN_WINDOW_SIZE];
        timestamp_.pop_back();

        delete(preintegrates_[FEN_WINDOW_SIZE]);
        preintegrates_[FEN_WINDOW_SIZE] = nullptr;

        slideNewFrame();
    }
}

bool Estimator::slideOldFrame() {
    // remove the oldest frame's feature
    if (initial_) {
        // change the feature depth when margin frame 
        // is reference frame of the feature
        Quaternionf Rc1c0 = RS_[1].inverse()*RS_[0];          // c1_R_c0 = c1_R_w * w_R_c0
        Vector3f    tc1c0 = RS_[1].inverse()*(PS_[0]-PS_[1]); // c1_P_c0 = c1_R_w * (w_P_c0 - w_P_c1)

        feature_manager_.removeOldestFrame(Rc1c0, tc1c0);
    }
    else {
        // do not change the feature depth
        feature_manager_.removeFrame(0);
    }
    return true;
}

bool Estimator::slideNewFrame() {
    feature_manager_.removeFrame(FEN_WINDOW_SIZE-1);
    return true;
}

void Estimator::vector2double() {
    // BA
    int  i = 0;
    for (i = 0; i <= FEN_WINDOW_SIZE; i++) {
        pose_params[i][0] = PS_[i].x();            
        pose_params[i][1] = PS_[i].y();            
        pose_params[i][2] = PS_[i].z();            
        pose_params[i][3] = RS_[i].x();
        pose_params[i][4] = RS_[i].y();
        pose_params[i][5] = RS_[i].z();
        pose_params[i][6] = RS_[i].w();

        motion_params[i][0] = VS_[i].x();
        motion_params[i][1] = VS_[i].y();
        motion_params[i][2] = VS_[i].z();
        motion_params[i][3] = BAS_[i].x();
        motion_params[i][4] = BAS_[i].y();
        motion_params[i][5] = BAS_[i].z();
        motion_params[i][6] = BGS_[i].x();
        motion_params[i][7] = BGS_[i].y();
        motion_params[i][8] = BGS_[i].z();
    }

    // feature point
    i = 0;
    auto &all_ftr = feature_manager_.all_ftr_;
    for (auto &id_ftr : all_ftr) {
        int id       = id_ftr.first;
        Feature* ftr = id_ftr.second;

        if (   ftr->size() <= 2 
            || ftr->ref_frame_id_ >= FEN_WINDOW_SIZE-2
            || ftr->inv_d_ <= 0) {
            continue;
        }

        point_params[i] = ftr->inv_d_;
        i++;
    }
}

void Estimator::solveOptimize() {
    // get all optimate variables
    vector2double();

    // ceres problem
    ceres::Problem problem;

    // add pose and motion
    for (int i = 0; i <= FEN_WINDOW_SIZE; i++) {
        ceres::LocalParameterization* pose_local = new PoseLocalParameter(); 
        problem.AddParameterBlock(pose_params[i], POSE_SIZE, pose_local);
        problem.AddParameterBlock(motion_params[i], MOTION_SIZE);
    }

    // add imu constraint
    for (int i = 1; i <= FEN_WINDOW_SIZE; i++) {
        Inertial_Factor* iner_factor = new Inertial_Factor(preintegrates_[i]);
        problem.AddResidualBlock(iner_factor, NULL, vector<double*>{
            pose_params[i-1], motion_params[i-1],
            pose_params[i-0], motion_params[i-0],
        });
    }

    // add feature constraint


    // optimize

}
