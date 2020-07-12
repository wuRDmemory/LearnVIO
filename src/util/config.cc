#include "../../include/util/config.h"
#include "../../include/util/log.h"

string CONFIG_PATH;
string IMAGE_TOPIC;
string IMU_TOPIC;
vector<string> CAM_NAMES;
string FISHEYE_MASK;

int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
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
float INIT_DEPTH;

double F_THRESHOLD;

Vector3f         Gw;
vector<Matrix3d> Rics;
vector<Vector3d> tics;

bool readParameters(string config_path, string vins_path) {
    string config_file_path;
    string vins_folder_path;

    config_file_path = config_path;
    vins_folder_path = vins_path;

    CONFIG_PATH = config_file_path;
    
    // load config file
    cv::FileStorage fs;
    fs.open(config_file_path, cv::FileStorage::READ);
    
    if (!fs.isOpened()) {
        LOGE("CAN OPEN CONFIG FILE!!!");
        return false;
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

    {   // read extrinsic parameter
        cv::Mat cv_R, cv_T;

        fs["extrinsicRotation"]    >> cv_R;
        fs["extrinsicTranslation"] >> cv_T;

        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);

        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        Rics.push_back(eigen_R);
        tics.push_back(eigen_T);

        cout << "Extrinsic Ric: " << endl << "\t" << Rics[0] << endl;
        cout << "Extrinsic tic: " << endl << "\t" << tics[0].transpose() << endl;
    }

    if (FISHEYE == 1)
        FISHEYE_MASK = vins_folder_path + "/config/fisheye_mask.jpg";

    CAM_NAMES.push_back(config_file_path);

    WINDOW_SIZE     = 20;
    STEREO_TRACK    = false;
    FOCAL_LENGTH    = 460;
    PUB_THIS_FRAME  = false;
    INIT_DEPTH      = 1;
    Gw              = Vector3f(0, 0, 9.81);

    if (FREQ == 0)
        FREQ = 100;

    fs.release();

    return true;
}
