#include "../include/estimator.h"
#include "../include/util/utils.h"
#include "../include/util/log.h"
#include "../include/util/config.h"
#include "../include/initial/initial.h"
#include "../include/visual/globalsfm.h"
#include "../include/visual/project_factor.h"
#include "../include/inertial/inertial_factor.h"

Estimator::Estimator() : 
    first_imu_(true), initial_(false), has_first_(true) {
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
    has_first_   = true;

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

    Gw.setZero();

    feature_manager_.clear();

    FILE* fp = fopen("/home/ubuntu/catkin_ws/src/learn_vio/states.txt", "w");
    fclose(fp);
}

void Estimator::processImu(double dt, const Vector3d& accl, const Vector3d& gyro) {
    if (first_imu_) {
        first_imu_ = false;
        accl_0_ = accl;
        gyro_0_ = gyro;
    }

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
        Vector3d gyro_mid;
        Vector3d accl_b0, accl_b1, accl_mid;
        
        gyro_mid = (gyro + gyro_0_)*dt/2;
        accl_b0  = RS_[frame_count_]*(accl_0_-BAS_[frame_count_]);
        
        RS_[frame_count_] *= vec2quat(gyro_mid);
        RS_[frame_count_].normalize();
        
        accl_b1  = RS_[frame_count_]*(accl-BAS_[frame_count_]);

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
            if ( success ) {
                // initial success 
                initial_ = true;
                solveOdometry();
                slideWindow(margin_old);
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
        solveOdometry();
        slideWindow(margin_old);
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

    Matrix3d Rwc;
    Vector3d twc;
    Matrix3d Rbc = Rics[0];
    Vector3d tbc = tics[0];
    Matrix3d Rcw[FEN_WINDOW_SIZE+1];
    Vector3d tcw[FEN_WINDOW_SIZE+1];
    for (int i = 0; i <= FEN_WINDOW_SIZE; i++) {
        Rwc = RS_[i].toRotationMatrix()*Rbc; // w_R_c = w_R_b*b_R_c
        twc = PS_[i] + RS_[i]*tbc;           // w_t_c = w_t_bk + w_R_bk*bk_t_ck

        Rcw[i] = Rwc.transpose();
        tcw[i] = Rwc.transpose()*twc*-1;
    }

    int new_ftr_cnt = feature_manager_.trianglesNew(Rcw, tcw);
    LOGD("[new ftr] Triangle new ldmk %d", new_ftr_cnt);

    solveOptimize();

    return true;
}

bool Estimator::solveNewFrame(map<int, Feature*> &all_features, Matrix3d Rwb, Vector3d twb) {
    vector<cv::Point3d> point3d;
    vector<cv::Point2f> point2d;

    Matrix3d Rbc = Rics[0];
    Vector3d tbc = tics[0];

    for (auto &id_ftr : all_features) {
        int       id = id_ftr.first;
        Feature* ftr = id_ftr.second;

        if (!ftr->contains(FEN_WINDOW_SIZE)) {
            continue;
        }

        int ref_id = ftr->ref_frame_id_;
        Matrix3d Rwbr = RS_[ref_id].toRotationMatrix();
        Vector3d twbr = PS_[ref_id];

        Matrix3d Rwcr;
        Vector3d twcr;
        cvtPoseFromBodyToCamera(Rwbr, twbr, Rbc, tbc, Rwcr, twcr);

        Vector3d Pw   = Rwcr*(ftr->vis_fs_[0]/ftr->inv_d_) + twcr;
        Vector3d pc   = ftr->getF(FEN_WINDOW_SIZE);
        assert(pc == ftr->vis_fs_.back());

        point3d.emplace_back(Pw.x(), Pw.y(), Pw.z());
        point2d.emplace_back(pc.x(), pc.y());
    }

    Matrix3d Rwc;
    Vector3d twc;
    cvtPoseFromBodyToCamera(Rwb, twb, Rbc, tbc, Rwc, twc);

    Matrix3d Rcw = Rwc.transpose();
    Vector3d tcw = Rcw*twc*-1;

    cv::Mat cvRcw, rvec, tvec;
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);

    cv::eigen2cv(Rcw,  cvRcw);
    cv::eigen2cv(tcw,  tvec);
    cv::Rodrigues(cvRcw, rvec);

    cv::solvePnP(point3d, point2d, K, cv::Mat(), rvec, tvec, true, cv::SOLVEPNP_EPNP);

    cv::Rodrigues(rvec, cvRcw);
    cv::cv2eigen(cvRcw, Rcw);
    cv::cv2eigen(tvec,  tcw);

    Rwc = Rcw.transpose();
    twc = (Rcw.transpose()*tcw*-1);

    cvtPoseFromCameraToBody(Rwc, twc, Rbc, tbc, Rwb, twb);

    return true;
}

bool Estimator::structInitial() {

    {   // check imu 
        map<double, FrameStruct>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_frames_.begin(), frame_it++; frame_it != all_frames_.end(); frame_it++) {
            double dt = frame_it->second.preintegrate_->sum_dt_;
            Vector3d tmp_g = frame_it->second.preintegrate_->delta_v_/dt;
            sum_g += tmp_g;
        }
        
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_frames_.size() - 1);
        double var = 0;
        for (frame_it = all_frames_.begin(), frame_it++; frame_it != all_frames_.end(); frame_it++) {
            double dt = frame_it->second.preintegrate_->sum_dt_;
            Vector3d tmp_g = frame_it->second.preintegrate_->delta_v_ / dt;
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
            RS_[i] = Quaterniond(qc0b[i]);
            PS_[i] = (pc0b[i]-pc0b[0]);
            VS_[i] = (vc0b[i]);

            BAS_[i] = all_frames[i]->bias_a_;
            BGS_[i] = all_frames[i]->bias_g_;

            preintegrates_[i]->reintegrate(Vector3d(0, 0, 0), BGS_[i]);
        }
    }



    // rotate the c0 to world coordinate
    Matrix3d Rwc0 = gravity2Rnb(g_c0);
    Matrix3d Rwb0 = Rwc0*Matrix3d(RS_[0]);
    // get the w_R_b0's yaw angle
    float    yaw   = Rnb2ypr(Rwb0).x(); // w_R_c0*c0_R_b0
    Matrix3d Rb0w  = ypr2Rnb(Vector3d{-yaw, 0, 0});
    
    Rwc0  = Rb0w*Rwc0; // correct yaw, regard b0's yaw as w's yaw
    Gw = Rwc0*g_c0;

    // change world frame from c0 to world
    for (int i = 0; i < N; i++) {
        RS_[i] = Rwc0*RS_[i];
        PS_[i] = Rwc0*PS_[i];
        VS_[i] = Rwc0*VS_[i];
    }

    Vector3d R0ypr = Rnb2ypr(Matrix3d(RS_[0]));
    LOGW("[Align] Rwc0: %f, %f, %f", R0ypr.x(), R0ypr.y(), R0ypr.z());
    LOGW("[Align] final Grivaty: %f, %f, %f", Gw(0), Gw(1), Gw(2));

    {   // re-triangle all features
        Matrix3d qcw[N];
        Vector3d tcw[N];
        Matrix3d Rwc;
        Vector3d twc;

        Matrix3d Ric = Rics[0];
        Vector3d tic = tics[0];

        int i = 0;
        for (i = 0; i < N; i++) {
            Rwc = RS_[i].toRotationMatrix()*Ric;      // c0_R_c = c0_R_bk*bk_R_ck
            twc = PS_[i] + RS_[i]*tic;                // c0_P_c = c0_P_bk + c0_R_bk*bk_P_ck
            
            qcw[i] = Rwc.transpose();
            tcw[i] = Rwc.transpose()*twc*-1;
        }

        feature_manager_.trianglesInitial(qcw, tcw);
    }

    // {
    //     FILE* file;
    //     file = fopen("./poses.txt", "w");
    //     for (int i = 0; i < N; i++) {
    //         Vector3d ypr = Rnb2ypr(Matrix3d(RS_[i]));
    //         fprintf(file, "%lf\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", 
    //                 timestamp_[i], ypr.x(), ypr.y(), ypr.z(),
    //                             PS_[i].x(), PS_[i].y(), PS_[i].z(),
    //                             VS_[i].x(), VS_[i].y(), VS_[i].z());
    //     }
    //     fclose(file);
    // }

    // {
    //     FILE* fp;
    //     fp = fopen("./motion.txt", "w");
    //     for (int i = 0; i <= FEN_WINDOW_SIZE; i++) {
    //         fprintf(fp, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", i, 
    //                 VS_[i][0], VS_[i][1], VS_[i][2], 
    //                 BAS_[i][0], BAS_[i][1], BAS_[i][2], 
    //                 BGS_[i][0], BGS_[i][1], BGS_[i][2]);
    //     }        
    //     fclose(fp);

    // }

    return true;
}

bool Estimator::slideWindow(bool margin_old) {
    assert(frame_count_ == FEN_WINDOW_SIZE);
    if (margin_old) {
        // margin oldest frame
        double t0 = timestamp_[0];

        // change the depth first
        // because here we need the X0 and X1
        slideOldFrame();
        
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

        has_first_ = false;
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
        Matrix3d Rwc0, Rwc1;
        Vector3d twc0, twc1;
        cvtPoseFromBodyToCamera(RS_[0].toRotationMatrix(), PS_[0], Rics[0], tics[0], Rwc0, twc0);
        cvtPoseFromBodyToCamera(RS_[1].toRotationMatrix(), PS_[1], Rics[0], tics[0], Rwc1, twc1);

        Matrix3d Rc1c0 = Rwc1.transpose()*Rwc0;        // c1_R_c0 = c1_R_w * w_R_c0
        Vector3d tc1c0 = Rwc1.transpose()*(twc0-twc1); // c1_P_c0 = c1_R_w * (w_P_c0 - w_P_c1)

        feature_manager_.removeOldestFrame(Rc1c0, tc1c0);
    }
    else {
        // do not change the feature depth
        feature_manager_.removeOldestFrame();
    }
    return true;
}

bool Estimator::slideNewFrame() {
    feature_manager_.removeNewFrame(FEN_WINDOW_SIZE);
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
    point_count = i;
}

void Estimator::solveOptimize() {
    // get all optimate variables
    vector2double();

    // ceres problem
    ceres::Problem problem;
    // ceres::LossFunction* loss = new ceres::HuberLoss(1.0);
    ceres::LossFunction* loss = new ceres::CauchyLoss(1.0);

    // add pose and motion
    for (int i = 0; i <= FEN_WINDOW_SIZE; i++) {
        ceres::LocalParameterization* pose_local = new PoseLocalParameter(); 
        problem.AddParameterBlock(pose_params[i],   POSE_SIZE,   pose_local);
        problem.AddParameterBlock(motion_params[i], MOTION_SIZE);
    }

    // add imu constraint
    for (int i = 1; i <= FEN_WINDOW_SIZE; i++) {
        Inertial_Factor* iner_factor = new Inertial_Factor(i-1, i, preintegrates_[i]);
        problem.AddResidualBlock(iner_factor, NULL, 
            pose_params[i-1], motion_params[i-1], pose_params[i], motion_params[i]);
    }

    // add feature constraint
    VisualCost::sqrt_info_ = FOCAL_LENGTH/1.5*Matrix2d::Identity();
    VisualCost::sum_t_     = 0;
    VisualCost::Rbc        = Rics[0];
    VisualCost::tbc        = tics[0];

    // cv::Mat show(ROW, COL, CV_8UC3, cv::Scalar::all(0));

    int all_vis_edge = 0;
    int i            = 0;
    auto &all_ftr    = feature_manager_.all_ftr_;
    for (auto &id_ftr : all_ftr) {
        int id       = id_ftr.first;
        Feature* ftr = id_ftr.second;

        if (   ftr->size() <= 2 
            || ftr->ref_frame_id_ >= FEN_WINDOW_SIZE-2
            || ftr->inv_d_ <= 0) {
            continue;
        }

        // problem.AddParameterBlock(&point_params[i], 1);

        int ref_frame_id = ftr->ref_frame_id_;

        Vector3d ref_pt = ftr->vis_fs_[0];
        Vector3d cur_pt = ref_pt;

        // cv::Point p1(ref_pt(0)*FOCAL_LENGTH+COL/2, cur_pt(1)*FOCAL_LENGTH+ROW/2);
        // cv::Point p2;

        for (int j = 1; j < ftr->size(); j++) {
            cur_pt = ftr->vis_fs_[j];

            if (ref_frame_id + j > FEN_WINDOW_SIZE) {
                LOGE("[estimate] Feature(%d) maybe bad: %d+%d>%d", id, ref_frame_id, j, FEN_WINDOW_SIZE);
                assert(false);
            }

            // p2 = cv::Point(cur_pt(0)*FOCAL_LENGTH+COL/2, cur_pt(1)*FOCAL_LENGTH+ROW/2);
            // cv::circle(show, p1, 2,  cv::Scalar(0, 0, 255), 1);
            // cv::circle(show, p2, 2,  cv::Scalar(0, 0, 255), 1);
            // cv::line(  show, p1, p2, cv::Scalar(0, 255, 0), 1);
            // p1 = p2;

            VisualCost* cost = new VisualCost(ref_frame_id, ref_frame_id+j, ref_pt, cur_pt);
            problem.AddResidualBlock(cost, loss, 
                pose_params[ref_frame_id], pose_params[ref_frame_id+j], &point_params[i]);

            all_vis_edge++;
        }

        i++;
    }
    assert(i == point_count);
    LOGD("[solver] all visual edge: %d", all_vis_edge);
    
    // cv::imshow("point track", show);
    // cv::waitKey();

    // optimize
    Tick tic;

    ceres::Solver::Options options;
    options.linear_solver_type         = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations         = 10;
    options.max_solver_time_in_seconds = 1.0;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    
    cout << summary.BriefReport() << endl;
    LOGD("[solve] Iterations  : %d", static_cast<int>(summary.iterations.size()));
    LOGD("[solve] solver costs: %lf", tic.delta_time());

    // TODO: marginalize


    // double to vector
    double2vector();

    {
        FILE* fp = fopen("/home/ubuntu/catkin_ws/src/learn_vio/states.txt", "a");
        fprintf(fp, "\n");

        for (int i = 0; i <= FEN_WINDOW_SIZE; i++) {
            Vector3d ypr = Rnb2ypr(Matrix3d(RS_[i]));
            fprintf(fp, "%d\n%lf\t%lf\t%lf\t%lf\t%lf\t%lf \
                           \n%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", i, 
                    ypr(0), ypr(1), ypr(2), PS_[i][0], PS_[i][1], PS_[i][2],
                    VS_[i][0], VS_[i][1], VS_[i][2], BAS_[i][0], BAS_[i][1], BAS_[i][2], BGS_[i][0], BGS_[i][1], BGS_[i][2]);
        }

        fclose(fp);
    } 

    // {
    //     FILE* fp = fopen("./motion.txt", "a");
    //     fprintf(fp, "\n");

    //     for (int i = 0; i <= FEN_WINDOW_SIZE; i++) {
    //         fprintf(fp, "%d\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", i, 
    //                 VS_[i][0], VS_[i][1], VS_[i][2], 
    //                 BAS_[i][0], BAS_[i][1], BAS_[i][2], 
    //                 BGS_[i][0], BGS_[i][1], BGS_[i][2]);
    //     }

    //     fclose(fp);
    // } 
}

void Estimator::double2vector() {
    // correct yaw 
    Matrix3d Rw0w;  // w0_R_w
    {   // this place correct the world corrdination,
        // not  b0 corrdination
        Matrix3d Rw0b    = RS_[0].toRotationMatrix();
        Quaterniond Qwb  = Quaterniond(pose_params[0][6], 
                                     pose_params[0][3], 
                                     pose_params[0][4], 
                                     pose_params[0][5]);
        Matrix3d Rwb     = Qwb.toRotationMatrix();

        // attitude of w00 in b coordination
        // attitude of w0  in b coordination
        Vector3d ypr0 = Rnb2ypr<double>(Rw0b.transpose());
        Vector3d ypr  = Rnb2ypr<double>(Rwb.transpose() );

        if (abs(abs(ypr.x())-90) < 1.0 || abs(abs(ypr0.x())-90) < 1.0) {
            Rw0w = Rw0b*Rwb.transpose();
        }
        else {
            // only correct 
            double yw0 = ypr0.x();
            double yw  = ypr.x();

            // rotate all w0 to w00
            double diff_w = yw - yw0; // diff between w and w0 
            Rw0w = ypr2Rnb(Vector3d(diff_w, 0, 0)); // w0_R_w
        }
    }

    Vector3d Pw0b = PS_[0];

    int  i = 0;
    for (i = 0; i <= FEN_WINDOW_SIZE; i++) {
        // rotate all pose to w0 from w
        // Pw0b
        PS_[i] = Rw0w*Vector3d(pose_params[i][0]-pose_params[0][0],
                               pose_params[i][1]-pose_params[0][1],
                               pose_params[i][2]-pose_params[0][2]) + Pw0b;
        
        // Rw0b
        Quaterniond Rwb(pose_params[i][6], pose_params[i][3], 
                        pose_params[i][4], pose_params[i][5]);
        RS_[i] = (Quaterniond(Rw0w)*Rwb).normalized();

        // Vw0b
        Vector3d Vwb(motion_params[i][0], motion_params[i][1], motion_params[i][2]);
        VS_[i] = Rw0w*Vwb;
        
        // Bias accl
        BAS_[i].x() = motion_params[i][3];
        BAS_[i].y() = motion_params[i][4];
        BAS_[i].z() = motion_params[i][5];
        
        // Bias gyro
        BGS_[i].x() = motion_params[i][6];
        BGS_[i].y() = motion_params[i][7];
        BGS_[i].z() = motion_params[i][8];
    }

    // feature point
    Matrix3d Rbc = Rics[0];
    Vector3d tbc = tics[0];

    Matrix3d Rwc;
    Vector3d twc;

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

        ftr->inv_d_ = point_params[i];
        
        // build world 3D point
        const int j = ftr->ref_frame_id_;
        Vector3d f  = ftr->vis_fs_[0];

        Rwc = RS_[j].toRotationMatrix()*Rbc;          // w_R_c = w_R_b*b_R_c;
        twc = PS_[j] + RS_[j].toRotationMatrix()*tbc; // w_t_c = w_t_b + w_R_b*b_t_c

        ftr->pt3d_ = (Rwc*f/ftr->inv_d_ + twc); // w_P = w_R_c*Pc + w_t_c

        i++;
    }
    assert(i == point_count);
}


