#include "camera_base.h"

double CameraBase::reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const {
    Eigen::Vector2d p1, p2;

    spaceToPlane(P1, p1);
    spaceToPlane(P2, p2);

    return (p1 - p2).norm();
}

double CameraBase::reprojectionError(const vector<vector<cv::Point3f> >& objectPoints,
                          const vector<vector<cv::Point2f> >& imagePoints,
                          const vector<cv::Mat>& rvecs,
                          const vector<cv::Mat>& tvecs,
                          cv::OutputArray _perViewErrors) const {
    int imageCount = objectPoints.size();
    size_t pointsSoFar = 0;
    double totalErr = 0.0;

    bool computePerViewErrors = _perViewErrors.needed();
    cv::Mat perViewErrors;
    if (computePerViewErrors)
    {
        _perViewErrors.create(imageCount, 1, CV_64F);
        perViewErrors = _perViewErrors.getMat();
    }

    for (int i = 0; i < imageCount; ++i)
    {
        size_t pointCount = imagePoints.at(i).size();

        pointsSoFar += pointCount;

        std::vector<cv::Point2f> estImagePoints;
        projectPoints(objectPoints.at(i), rvecs.at(i), tvecs.at(i),
                      estImagePoints);

        double err = 0.0;
        for (size_t j = 0; j < imagePoints.at(i).size(); ++j)
        {
            err += cv::norm(imagePoints.at(i).at(j) - estImagePoints.at(j));
        }

        if (computePerViewErrors)
        {
            perViewErrors.at<double>(i) = err / pointCount;
        }

        totalErr += err;
    }

    return totalErr / pointsSoFar;
}

double CameraBase::reprojectionError(const Eigen::Vector3d& P,
                          const Eigen::Quaterniond& camera_q,
                          const Eigen::Vector3d& camera_t,
                          const Eigen::Vector2d& observed_p) const {
    Eigen::Vector3d P_cam = camera_q.toRotationMatrix()*P + camera_t;

    Eigen::Vector2d p;
    spaceToPlane(P_cam, p);

    return (p - observed_p).norm();
}

void CameraBase::projectPoints(const vector<cv::Point3f>& objectPoints,
                      const cv::Mat& rvec,
                      const cv::Mat& tvec,
                      std::vector<cv::Point2f>& imagePoints) const {
    // project 3D object points to the image plane
    imagePoints.reserve(objectPoints.size());

    //double
    cv::Mat R0;
    cv::Rodrigues(rvec, R0);

    Eigen::MatrixXd R(3,3);
    R << R0.at<double>(0,0), R0.at<double>(0,1), R0.at<double>(0,2),
         R0.at<double>(1,0), R0.at<double>(1,1), R0.at<double>(1,2),
         R0.at<double>(2,0), R0.at<double>(2,1), R0.at<double>(2,2);

    Eigen::Vector3d t;
    t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        const cv::Point3f& objectPoint = objectPoints.at(i);

        // Rotate and translate
        Eigen::Vector3d P;
        P << objectPoint.x, objectPoint.y, objectPoint.z;

        P = R * P + t;

        Eigen::Vector2d p;
        spaceToPlane(P, p);

        imagePoints.push_back(cv::Point2f(p(0), p(1)));
    }
}
