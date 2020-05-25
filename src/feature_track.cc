/*
 * @Author: your name
 * @Date: 2020-05-24 21:40:20
 * @LastEditTime: 2020-05-25 22:59:34
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
        ROS_DEBUG("FM ransac begin!!!");
        Tick tick;
        
        // convert point in image to normalized plane
        vector<cv::Point2f> un_prev_pts(prev_pts_.size());
        vector<cv::Point2f> un_curr_pts(curr_pts_.size());

        for (int i = 0; i < curr_pts_.size(); i++) {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}
