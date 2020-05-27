/*
 * @Author: your name
 * @Date: 2020-05-27 21:31:20
 * @LastEditTime: 2020-05-27 22:25:08
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /LearnVIO/camera/camera_base.h
 */ 
#pragma once

#include <iostream>

#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;
using namespace cv;

class CameraModel {
public:
    enum ModelType {
        PINHOLE = 1,
        NONE,  
    };

protected:
    int width_;
    int height_;
   
    string    name_;
    ModelType type_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    CameraModel() {}
    CameraModel(int width, int height, string name, ModelType type): 
        width_(width), height_(height), name_(name), type_(type) {}

    virtual ~CameraModel() {}

    virtual Vector3f
    cam2world(const float& x, const float& y) const = 0;

    /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
    virtual Vector3f
    cam2world(const Vector2f& px) const = 0;

    virtual Vector2f
    world2cam(const Vector3f& xyz_c) const = 0;

    /// projects unit plane coordinates to camera coordinates
    virtual Vector2f
    world2cam(const Vector2f& uv) const = 0;

    virtual float
    errorMultiplier2() const = 0;

    virtual float
    errorMultiplier()  const = 0;

    virtual Vector2f
    project2d(const Vector3f& xyz) const {
        return xyz.head<2>()/xyz(2);
    }

    virtual const Matrix3f& K()    const = 0;

    virtual const Matrix3f& invK() const = 0;
    
    virtual const Mat&      cvK()  const = 0;

    virtual const Mat&    cvinvK() const = 0;

    virtual int width()  const { return width_;  }
    virtual int height() const { return height_; }
};


typedef CameraModel* CameraModelPtr;
