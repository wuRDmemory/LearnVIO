#include "../include/estimator.h"
#include "../include/util/utils.h"
#include "../include/util/log.h"
#include "../include/util/config.h"
#include "../include/initial/initial.h"
#include "../include/visual/globalsfm.h"

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
    // 
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
        accl_b1   = RS_[frame_count_]*(accl-BAS_[frame_count_]);

        accl_mid  = (accl_b0 + accl_b1)/2 - g_;
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
    temp_preintegrate_ = new PreIntegrate(accl_0_, gyro_0_, BAS_[frame_count_], BGS_[frame_count_]);
    all_frames_.insert(make_pair(timestamp, this_frame));

    if (!initial_) {
        if (frame_count_ >= FEN_WINDOW_SIZE) {
            // enough frame
            // initialize
            bool success = structInitial();
        }
        else {
            frame_count_++;
        }
    }
    else {
        // initial has done
    }
}

bool Estimator::structInitial() {

    {   // check 
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

    Matrix3f Rcr;
    Vector3f tcr;
    int l = relativeRT(feature_manager_.all_ftr_, Rcr, tcr, FEN_WINDOW_SIZE);
    if ( l < 0 ) {
        LOGI("[struct initial] can not get good RT, failed!!");
        return false;
    }

    LOGI("[struct initial] get a good RT");

    vector<FrameStruct> all_frames;
    for (auto& pir : all_frames_) {
        all_frames.push_back(pir.second);
    }
    
    int ret = globalSFM(feature_manager_.all_ftr_, all_frames, Rcr, tcr, l);
    if ( ret == 0 ) {
        return false;
    }

    int i = 0;
    for (auto& pir : all_frames_) {
        pir.second = all_frames[i];
    }

    return true;
}
