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

using namespace std;
using namespace Eigen;

static bool first_feature = true;

static double last_imu_t = 0;

static condition_variable con;
static mutex imu_buf_lock;
static mutex feature_lock;
static mutex imu_state_lock;

static queue<sensor_msgs::PointCloudConstPtr> feature_buf;
static queue<sensor_msgs::ImuConstPtr>        imu_buf;

typedef pair<vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr> Data_Type;

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

vector<Data_Type>
getMeasures() {    
    vector<Data_Type> ret;

    return ret;
}
 
// main loop
void process() {
    while(true) {
        vector<Data_Type> measures; 
        unique_lock lock(imu_buf_lock);

        con.wait(lock, [&]{
            return (measures = getMeasures()).size() != 0;
        });

        // already handle the imu buffer
        lock.unlock();

        
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "front_end");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    readParameters(n);

    ros::Subscriber point_cloud_suber = n.subscribe("feature", 1000, pointCloudCallBack);


    
    return 1;
}
