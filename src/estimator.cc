#include "../include/estimator.h"
#include "../include/util/utils.h"
#include "../include/util/log.h"
#include "../include/util/config.h"

Estimator::Estimator() {
    first_imu_ = true;

    initial_   = false;

    frame_count_ = 0;

    feature_manager_ = new FeatureManager();

    preintegrates_.resize(FEN_WINDOW_SIZE+1);

    RS_.resize(FEN_WINDOW_SIZE+1);
    VS_.resize(FEN_WINDOW_SIZE+1);
    PS_.resize(FEN_WINDOW_SIZE+1);

    BAS_.resize(FEN_WINDOW_SIZE+1);
    BGS_.resize(FEN_WINDOW_SIZE+1);

    for (int i = 0; i <= FEN_WINDOW_SIZE; i++) {
        preintegrates_[i] = NULL;

        RS_[i].setIdentity();
        VS_[i].setZero();
        PS_[i].setZero();

        BAS_[i].setZero();
        BGS_[i].setZero();
    }

    g_.setZero();

}

Estimator::~Estimator() {
    ;
}

void Estimator::processImu(double dt, Vector3f accl, Vector3f gyro) {
    
    // LOGD("[estimate] FEN_WINDOW_SIZE: %d, frame id %d", FEN_WINDOW_SIZE, frame_count_); 
    // 
    if (preintegrates_[frame_count_] == NULL) {
        preintegrates_[frame_count_] = new PreIntegrate(accl, gyro, BAS_[frame_count_], BGS_[frame_count_]);
    } 

    if (frame_count_ != 0) {
        // first frame have no delta
        // so do not integrate

        preintegrates_[frame_count_]->push_back(dt, accl, gyro);

        // the prior R V P
        Vector3f gyro_mid;
        Vector3f accl_b0, accl_b1, accl_mid;
        
        gyro_mid = (gyro + gyro_0_)*dt/2;
        accl_b0  = RS_[frame_count_]*(accl_0_-BAS_[frame_count_]);
        
        RS_[frame_count_] *= vec2quat(gyro_mid);
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
    bool margin_old = feature_manager_->addNewFeatures(image, frame_count_);
    if (margin_old) {
        LOGI("[estimate] Margin old state, build new key frame");
    } 
    else {
        LOGI("[estimate] Margin new state");
    }

    LOGI("[estimate] Solving %d frame", frame_count_);
    LOGI("[estimate] Feature number : %d", feature_manager_->size());

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

    map<int, Feature*>& all_ftr = feature_manager_->all_ftr_;

    vector<Vector3f> ref_pts;
    vector<Vector3f> cur_pts;

    vector<Vector2f> ref_im_pts;
    vector<Vector2f> cur_im_pts;

    // scan from first frame
    int cur_frame_id = FEN_WINDOW_SIZE;
    for (int ref_frame_id = 0; ref_frame_id < FEN_WINDOW_SIZE; ref_frame_id++) {
        
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

        for (int id : ftr_ids) {
            ref_pts.push_back(all_ftr[id]->getF(ref_frame_id));
            cur_pts.push_back(all_ftr[id]->getF(cur_frame_id));

            ref_im_pts.push_back(all_ftr[id]->getUV(ref_frame_id));
            cur_im_pts.push_back(all_ftr[id]->getUV(cur_frame_id));
        }
    }    
    return true;
}
