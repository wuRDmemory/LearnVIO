#include "../include/estimator.h"
#include "../include/utils.h"

Estimator::Estimator() {
    frame_count_ = 0;

    preintegrates_.resize(FEN_WINDOW_SIZE, NULL);

    RS_.resize(FEN_WINDOW_SIZE, Quaternionf::Identity());
    VS_.resize(FEN_WINDOW_SIZE, Vector3f::Zero());
    PS_.resize(FEN_WINDOW_SIZE, Vector3f::Zero());

    BAS_.resize(FEN_WINDOW_SIZE, Vector3f::Zero());
    BGS_.resize(FEN_WINDOW_SIZE, Vector3f::Zero());

    g_.setZero();
}

Estimator::~Estimator() {
    ;
}


void Estimator::processImu(double dt, Vector3f accl, Vector3f gyro) {
    
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
    ;
}


