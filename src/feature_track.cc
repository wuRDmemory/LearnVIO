/*
 * @Author: your name
 * @Date: 2020-05-24 21:40:20
 * @LastEditTime: 2020-05-27 22:43:29
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /LearnVIO/src/feature_track.cc
 */ 
#include "../include/feature_track.h"
#include "../include/config.h"
#include "../include/tick.h"

int FeatureTrack::id = 0;


FeatureTrack::FeatureTrack() {
    ids_.clear();
    curr_pts_.clear();
    prev_pts_.clear();

    camera_ = NULL;
}

void FeatureTrack::readImage(const cv::Mat& image, double timestamp) {
    curr_image_ = image.clone();

    if (!curr_pts_.empty()) {
        curr_pts_.clear();

        vector<uchar> status;
        vector<float> error;
        cv::calcOpticalFlowPyrLK(prev_image_, curr_image_, prev_pts_, curr_pts_, status, error);

        for (int i = 0; i < (int)status.size(); i++)
        if  (status[i] && !isVisable(curr_pts_[i]))
            status[i] = 0;
        
        reduceVector(prev_pts_,  status);
        reduceVector(curr_pts_,  status);
        reduceVector(ids_,       status);
        reduceVector(track_cnt_, status);
    }

    // update track count
    for (int& n : track_cnt_)
        n++;
    
    // frequency control here
    if (PUB_THIS_FRAME) {
        // use fundamental matrix filte some oulier
        rejectWithF();

        // set the mask and get some new key points
        
    }

    prev_image_ = curr_image_.clone();
    prev_pts_   = move(curr_pts_);
}

bool FeatureTrack::isVisable(const cv::Point2f& pt) {
    const int b = 10;
    const int x = pt.x;
    const int y = pt.y;

    bool c1 = b < x && x < COL-10;
    bool c2 = b < y && y < ROW-10;
    
    return c1 && c2;
}

template<typename T>
void FeatureTrack::reduceVector(vector<T>& vec, vector<uchar>& status) {
    int j = 0;
    for (int i = 0; i < vec.size(); i++) {
        if (status[i]) {
            vec[j] = vec[i];
            j++;     
        }
    }
    vec.resize(j);
}

void FeatureTrack::rejectWithF() {
    if (curr_pts_.size() > 8)  {
        ROS_DEBUG("[reject] FM ransac begin!!!");
        Tick tick;
        
        // convert point in image to normalized plane
        vector<cv::Point2f> un_prev_pts(prev_pts_.size());
        vector<cv::Point2f> un_curr_pts(curr_pts_.size());

        Vector3f Pc;
        for (int i = 0; i < curr_pts_.size(); i++) {

            Pc = camera_->cam2world(prev_pts_[i].x, prev_pts_[i].y);
            Pc.x() = FOCAL_LENGTH * Pc.x() / Pc.z() + COL / 2.0;
            Pc.y() = FOCAL_LENGTH * Pc.y() / Pc.z() + ROW / 2.0;
            prev_un_pts_[i] = cv::Point2f(Pc.x(), Pc.y());

            Pc = camera_->cam2world(curr_pts_[i].x, curr_pts_[i].y);
            Pc.x() = FOCAL_LENGTH * Pc.x() / Pc.z() + COL / 2.0;
            Pc.y() = FOCAL_LENGTH * Pc.y() / Pc.z() + ROW / 2.0;
            curr_un_pts_[i] = cv::Point2f(Pc.x(), Pc.y());
        }
        
        vector<uchar> status;
        cv::findFundamentalMat(prev_un_pts_, curr_un_pts_, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        
        int size_a = curr_pts_.size();
        reduceVector(prev_pts_,    status);
        reduceVector(curr_pts_,    status);
        reduceVector(curr_un_pts_, status);
        reduceVector(prev_un_pts_, status);
        reduceVector(ids_,         status);
        reduceVector(track_cnt_,   status);

        ROS_DEBUG("[reject] FM ransac: %d -> %lu: %f", size_a, curr_pts_.size(), 1.0 * curr_pts_.size() / size_a);
        ROS_DEBUG("[reject] FM ransac costs: %lf ms", tick.delta_time());
    }
}
