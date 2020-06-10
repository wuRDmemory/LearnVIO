#include "../include/config.h"
#include "opencv2/opencv.hpp"

string CONFIG_PATH;
string IMAGE_TOPIC;
string IMU_TOPIC;
vector<string> CAM_NAMES;
string FISHEYE_MASK;

int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

float ACCL_N, GYRO_N;
float ACCL_BIAS_N, GYRO_BIAS_N;

int FEN_WINDOW_SIZE;

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


void readParameters(ros::NodeHandle& n) {
    string config_file_path;
    string vins_folder_path;
    config_file_path = getParameter<string>(n, "config_file");
    vins_folder_path = getParameter<string>(n, "vins_folder");

    CONFIG_PATH = config_file_path;
    
    // load config file
    cv::FileStorage fs;
    fs.open(config_file_path, cv::FileStorage::READ);
    
    if (!fs.isOpened()) {
        ROS_ERROR("CAN OPEN CONFIG FILE!!!");
        n.shutdown();
        return ;
    }


    fs["image_topic"] >> IMAGE_TOPIC;
    fs["imu_topic"]   >> IMU_TOPIC;
    MAX_CNT     = fs["max_cnt"];
    MIN_DIST    = fs["min_dist"];
    ROW         = fs["image_height"];
    COL         = fs["image_width"];
    FREQ        = fs["freq"];
    F_THRESHOLD = fs["F_threshold"];
    SHOW_TRACK  = fs["show_track"];
    EQUALIZE    = fs["equalize"];
    FISHEYE     = fs["fisheye"];

    ACCL_N      = fs["accl_noise"];
    GYRO_N      = fs["gyro_noise"];
    ACCL_BIAS_N = fs["accl_bias_noise"];
    GYRO_BIAS_N = fs["gyro_bias_noise"];


    if (FISHEYE == 1)
        FISHEYE_MASK = vins_folder_path + "/config/fisheye_mask.jpg";

    CAM_NAMES.push_back(config_file_path);

    WINDOW_SIZE    = 20;
    STEREO_TRACK   = false;
    FOCAL_LENGTH   = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    FEN_WINDOW_SIZE = 10;

    fs.release();
}
