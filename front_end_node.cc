#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "include/config.h"
#include "include/tick.h"
#include "include/estimator.h"

using namespace std;
using namespace Eigen;

typedef pair<vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr> Data_Type;

static bool first_feature = true;

static double current_time = -1;
static double last_imu_t = 0;

static Estimator estimator;

static condition_variable con;
static mutex imu_buf_lock;
static mutex feature_lock;
static mutex imu_state_lock;
static mutex estimate_lock;

static queue<sensor_msgs::PointCloudConstPtr> feature_buf;
static queue<sensor_msgs::ImuConstPtr>        imu_buf;


// feature callback
void pointCloudCallBack(const sensor_msgs::PointCloudConstPtr& feature_msg) {
    if (first_feature) {
        first_feature = false;
        return ;
    }

    feature_lock.lock();
    feature_buf.push(feature_msg);
    feature_lock.unlock();
    con.notify_one();
}

// imu callback
void imuCallBack(const sensor_msgs::ImuConstPtr &imu_msg) {
    if (imu_msg->header.stamp.toSec() <= last_imu_t) {
        ROS_WARN("imu message in disorder!");
        return;
    }

    imu_buf_lock.lock();
    imu_buf.push(imu_msg);
    imu_buf_lock.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        lock_guard<mutex> lg(imu_state_lock);
        // predict(imu_msg);

        // std_msgs::Header header = imu_msg->header;
        // header.frame_id = "world";
        // if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        //     pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

vector<Data_Type> getMeasures() {    
    vector<Data_Type> ret;
    while (true) {
        if (imu_buf.empty() || feature_buf.empty()) {
            break;
        }

        // check timestamp of imu and feature
        // 1. newest imu is early than oldest feature
        //    IMU ts:      |----------|
        //    feature ts:                |------|
        if (imu_buf.back()->header.stamp.toSec() <= feature_buf.front()->header.stamp.toSec()) {
            ROS_WARN("waiting for IMU data!");
            break;
        }

        // 2. newest feature is early than oldest imu
        //    IMU ts:                |-------|
        //    feature ts:  |------|
        if (feature_buf.back()->header.stamp.toSec() <= imu_buf.back()->header.stamp.toSec()) {
            ROS_WARN("throw this feature frame!");
            break;
        }

        // normal case
        sensor_msgs::PointCloudConstPtr feature;
        feature = feature_buf.front();
        feature_buf.pop();

        vector<sensor_msgs::ImuConstPtr> imus;
        imus.reserve(10);
        while (imu_buf.front()->header.stamp.toSec() < feature->header.stamp.toSec()) {
            imus.push_back(imu_buf.front());
            imu_buf.pop();
        }
        imus.push_back(imu_buf.front());
        imu_buf.pop();

        if (imus.empty()) {
            ROS_WARN("IMU empty between two frame");
        }

        ret.push_back(make_pair(imus, feature));
    }

    return ret;
}
 
// main loop
void process() {
    while(true) {
        vector<Data_Type> measures; 
        unique_lock<mutex> lock(imu_buf_lock);

        con.wait(lock, [&]{
            return (measures = getMeasures()).size() != 0;
        });

        // already handle the imu buffer
        lock.unlock();

        estimate_lock.lock();
        for (Data_Type& measure : measures) {
            auto feature = measure.second;
            auto imus    = measure.first;

            float feature_time = feature->header.stamp.toSec();            
            float ax, ay, az, gx, gy, gz;
            float dt;

            for (auto& imu : imus) {
                double t     = imu->header.stamp.toSec();
                if (t <= feature_time) { 
                    if (current_time < 0)
                        current_time = t;

                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    ax = imu->linear_acceleration.x;
                    ay = imu->linear_acceleration.y;
                    az = imu->linear_acceleration.z;
                    gx = imu->angular_velocity.x;
                    gy = imu->angular_velocity.y;
                    gz = imu->angular_velocity.z;
                    estimator.processImu(dt, Vector3f(ax, ay, az), Vector3f(gx, gy, gz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else {
                    // interpolation
                    // for last imu data
                    double dt_1 = feature_time - current_time;
                    double dt_2 = t - feature_time;
                    current_time = feature_time;

                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);

                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    ax = w1 * ax + w2 * imu->linear_acceleration.x;
                    ay = w1 * ay + w2 * imu->linear_acceleration.y;
                    az = w1 * az + w2 * imu->linear_acceleration.z;
                    gx = w1 * gx + w2 * imu->angular_velocity.x;
                    gy = w1 * gy + w2 * imu->angular_velocity.y;
                    gz = w1 * gz + w2 * imu->angular_velocity.z;
                    estimator.processImu(dt, Vector3f(ax, ay, az), Vector3f(gx, gy, gz));
                }
            }

            Tick tick;
            Image_Type image;
            for (unsigned int i = 0; i < feature->points.size(); i++) {
                int feature_id = feature->channels[0].values[i];
                
                vector<double> data(7, 0);
                data[0] = feature->points[i].x;
                data[1] = feature->points[i].y;
                data[2] = feature->points[i].z;
                data[3] = feature->channels[1].values[i];
                data[4] = feature->channels[2].values[i];
                data[5] = feature->channels[3].values[i];
                data[6] = feature->channels[4].values[i];
                ROS_ASSERT(data[2] == 1);

                image.insert(make_pair(feature_id, make_pair(0, data)));
            }

            unique_lock<mutex> lock(estimate_lock);
            estimator.processImage(feature->header.stamp.toSec(), image);

            ROS_DEBUG("[process] estimator eclipse : %lf", tick.delta_time());
        }
        
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "front_end");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    readParameters(n);

    ros::Subscriber point_cloud_suber = n.subscribe("/feature", 1000, pointCloudCallBack);
    ros::Subscriber imu_suber         = n.subscribe(IMU_TOPIC, 2000, imuCallBack, ros::TransportHints().tcpNoDelay());
    
    ros::spin();
    return 1;
}
