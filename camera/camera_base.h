#pragma once

#include <iostream>
#include <vector>
#include <map>

#include "opencv2/opencv.hpp"
#include "Eigen/Core"
#include "Eigen/Dense"

using namespace std;
using namespace cv;

class CameraBase {
public:
    enum ModelType
    {
        KANNALA_BRANDT,
        MEI,
        PINHOLE,
        SCARAMUZZA,
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ModelType model_type_;
    string    camera_name_;
    int       width_;
    int       height_;

    cv::Mat   mask_;

public:

    // virtual void estimateIntrinsics(const cv::Size& boardSize,
    //                                 const std::vector< std::vector<cv::Point3f> >& objectPoints,
    //                                 const std::vector< std::vector<cv::Point2f> >& imagePoints) = 0;
    // virtual void estimateExtrinsics(const std::vector<cv::Point3f>& objectPoints,
    //                                 const std::vector<cv::Point2f>& imagePoints,
    //                                 cv::Mat& rvec, cv::Mat& tvec) const;

    // Lift points from the image plane to the sphere
    virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
    //%output P

    // Lift points from the image plane to the projective space
    virtual void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
    //%output P

    // Projects 3D points to the image plane (Pi function)
    virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const = 0;
    //%output p

    // Projects 3D points to the image plane (Pi function)
    // and calculates jacobian
    //virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p,
    //                          Eigen::Matrix<double,2,3>& J) const = 0;
    //%output p
    //%output J

    virtual void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const = 0;
    //%output p

    //virtual void initUndistortMap(cv::Mat& map1, cv::Mat& map2, double fScale = 1.0) const = 0;
    virtual cv::Mat initUndistortRectifyMap(cv::Mat& map1, cv::Mat& map2,
                                            float fx = -1.0f, float fy = -1.0f,
                                            cv::Size imageSize = cv::Size(0, 0),
                                            float cx = -1.0f, float cy = -1.0f,
                                            cv::Mat rmat = cv::Mat::eye(3, 3, CV_32F)) const = 0;

    virtual int parameterCount(void) const = 0;

    virtual void readParameters(const std::vector<double>& parameters) = 0;

    virtual void writeParameters(std::vector<double>& parameters) const = 0;



    virtual void writeParametersToYamlFile(const std::string& filename) const = 0;



    virtual std::string parametersToString(void) const = 0;

    /**
     * \brief Calculates the reprojection distance between points
     *
     * \param P1 first 3D point coordinates
     * \param P2 second 3D point coordinates
     * \return euclidean distance in the plane
     */
    double reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const;

    double reprojectionError(const std::vector< std::vector<cv::Point3f> >& objectPoints,
                             const std::vector< std::vector<cv::Point2f> >& imagePoints,
                             const std::vector<cv::Mat>& rvecs,
                             const std::vector<cv::Mat>& tvecs,
                             cv::OutputArray perViewErrors = cv::noArray()) const;

    double reprojectionError(const Eigen::Vector3d& P,
                             const Eigen::Quaterniond& camera_q,
                             const Eigen::Vector3d& camera_t,
                             const Eigen::Vector2d& observed_p) const;

    void projectPoints(const std::vector<cv::Point3f>& objectPoints,
                       const cv::Mat& rvec,
                       const cv::Mat& tvec,
                       std::vector<cv::Point2f>& imagePoints) const;
};
