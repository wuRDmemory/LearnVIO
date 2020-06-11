#include <iostream>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include "camera/camera_factory.h"
#include "camera/camera_base.h"
#include "include/util/config.h"
#include "include/util/tick.h"
#include "include/feature_track.h"

using namespace std;

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::ImageConstPtr> img_buf;

ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

int    pub_count        = 1;
bool   first_image_flag = true;
bool   init_pub         = 0;
double last_image_time  = 0;
double first_image_time = 0;

CameraModelPtr camera;
FeatureTrack tracker;

template <typename T> 
T getParameter(ros::NodeHandle& n, string name) {
    T ans;
    if (n.getParam(name, ans)) {
        ROS_INFO_STREAM("load parameter " << name << " : " << ans);
    }
    else {
        ROS_ERROR_STREAM("CAN NOT LOAD PARAMETER: " << name);
        n.shutdown();
    }

    return ans;
}


void img_callback(const sensor_msgs::ImageConstPtr &img_msg) {
    
    if(first_image_flag) {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time  = img_msg->header.stamp.toSec();
        return;
    }

    // detect unstable camera stream
    if (   img_msg->header.stamp.toSec() - last_image_time > 1.0 
        || img_msg->header.stamp.toSec() < last_image_time) {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag  = true; 
        last_image_time   = 0;
        pub_count         = 1;

        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }

    last_image_time = img_msg->header.stamp.toSec();
    
    // frequency control
    if (round(pub_count/(img_msg->header.stamp.toSec()-first_image_time)) <= FREQ) {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(pub_count/(img_msg->header.stamp.toSec()-first_image_time)-FREQ) < 0.01*FREQ) {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count        = 0;
        }
    }
    else {
        PUB_THIS_FRAME = false;
    }

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1") {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else {
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }

    cv::Mat image = ptr->image.rowRange(0, ROW);

    ROS_DEBUG("[read image] processing camera");
    tracker.readImage(image, img_msg->header.stamp.toSec());

#if SHOW_UNDISTORTION
    trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";
        
        // publish 
        auto& curr_un_pts = tracker.curr_un_pts_;
        auto& curr_pts    = tracker.curr_pts_;
        auto& curr_ids    = tracker.ids_;
        auto& curr_velocity  = tracker.pts_velocity_;
        auto& curr_track_cnt = tracker.track_cnt_;

        for (int j = 0; j < curr_ids.size(); j++) {
            if (curr_track_cnt[j] > 1) {
                int p_id = curr_ids[j];

                geometry_msgs::Point32 p;
                p.x = curr_un_pts[j].x;
                p.y = curr_un_pts[j].y;
                p.z = 1;

                feature_points->points.push_back(p);
                id_of_point.values.push_back(p_id);

                u_of_point.values.push_back(curr_pts[j].x);
                v_of_point.values.push_back(curr_pts[j].y);

                velocity_x_of_point.values.push_back(curr_velocity[j].x);
                velocity_y_of_point.values.push_back(curr_velocity[j].y);
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());

        // skip the first image; since no optical speed on frist image
        if (!init_pub) {
            init_pub = 1;
        }
        else {
            pub_img.publish(feature_points);
        }

        // add new when published current tracked features
        tracker.addPoints();

        if (SHOW_TRACK) {
            cv::Mat show_image;
            cv::cvtColor(image, show_image, cv::COLOR_GRAY2RGB);

            for (int j = 0; j < tracker.curr_pts_.size(); j++) {
                
                double len = std::min(1.0, (1.0*tracker.track_cnt_[j])/WINDOW_SIZE);
                cv::circle(show_image, tracker.curr_pts_[j], 2, cv::Scalar(255*(1-len), 0, 255*len), 2);
                //draw speed line
                /*
                Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                Vector3d tmp_prev_un_pts;
                tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                tmp_prev_un_pts.z() = 1;
                Vector2d tmp_prev_uv;
                trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                */
                //char name[10];
                //sprintf(name, "%d", trackerData[i].ids[j]);
                //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            }
            
            cv::imshow("vis", show_image);
            cv::waitKey(5);
            pub_match.publish(ptr->toImageMsg());
        }
    }
    ROS_INFO("------------------------");
    
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    
    string config_path = getParameter<string>(n, "config_file");
    string vins_path   = getParameter<string>(n, "vins_folder");
    
    readParameters(config_path, vins_path);

    camera = CameraFactory::buildModuleFromConfigFile(CONFIG_PATH);
    tracker.setCamera(camera);

    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    pub_img     = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match   = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);

    ros::spin();
    return 0;
}


