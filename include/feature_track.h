/*
 * @Author: your name
 * @Date: 2020-05-24 21:31:12
 * @LastEditTime: 2020-05-28 13:31:38
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Editin
 * @FilePath: /LearnVIO/include/feature_track.h
 */ 
#pragma once

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "../camera/camera_base.h"

using namespace std;
using namespace cv;

class FeatureTrack {
public:
    static int id;

public:
    double curr_timestamp_;
    double prev_timestamp_;

    cv::Mat mask_;
    cv::Mat fisheye_mask_;
    cv::Mat prev_image_;
    cv::Mat curr_image_;

    vector<Point2f> curr_pts_;
    vector<Point2f> prev_pts_;

    vector<Point2f>    curr_un_pts_;
    vector<Point2f>    prev_un_pts_;
    map<int, Point2f>  curr_id_pts_;
    map<int, Point2f>  prev_id_pts_;

    
    vector<Point2f> pts_velocity_;

    vector<Point2f> pts_new_;
    
    vector<int> ids_;
    vector<int> track_cnt_;

    CameraModelPtr camera_;

public:
    FeatureTrack();
    ~FeatureTrack();

    void readImage(const cv::Mat& image, double timestamp);
    
    void addPoints();

    void setCamera(CameraModelPtr camera) { camera_ = camera; }
    
private:
    bool isVisable(const cv::Point2f& pt);

    template<typename T>
    void reduceVector(vector<T>& vec, vector<uchar>& status);

    void rejectWithF();

    void setMask();

    void undistortPoints();

};
