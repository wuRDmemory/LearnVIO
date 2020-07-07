#include "../include/visualizor.h"
#include "../include/util/config.h"

void registerPub(ros::NodeHandle &n)
{
    pub_path     = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header)
{
    // if (estimator.initial_)
    // {
    //     nav_msgs::Odometry odometry;
    //     odometry.header = header;
    //     odometry.header.frame_id = "world";
    //     odometry.child_frame_id  = "world";

    //     Quaternionf tmp_Q;
    //     tmp_Q = estimator.RS_[WINDOW_SIZE]);
    //     odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
    //     odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
    //     odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
    //     odometry.pose.pose.orientation.x = tmp_Q.x();
    //     odometry.pose.pose.orientation.y = tmp_Q.y();
    //     odometry.pose.pose.orientation.z = tmp_Q.z();
    //     odometry.pose.pose.orientation.w = tmp_Q.w();
    //     odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
    //     odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
    //     odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
    //     pub_odometry.publish(odometry);

    //     geometry_msgs::PoseStamped pose_stamped;
    //     pose_stamped.header = header;
    //     pose_stamped.header.frame_id = "world";
    //     pose_stamped.pose = odometry.pose.pose;
    //     path.header = header;
    //     path.header.frame_id = "world";
    //     path.poses.push_back(pose_stamped);
    //     pub_path.publish(path);

    //     Vector3d correct_t;
    //     Vector3d correct_v;
    //     Quaterniond correct_q;
    //     correct_t = estimator.drift_correct_r * estimator.Ps[WINDOW_SIZE] + estimator.drift_correct_t;
    //     correct_q = estimator.drift_correct_r * estimator.Rs[WINDOW_SIZE];
    //     odometry.pose.pose.position.x = correct_t.x();
    //     odometry.pose.pose.position.y = correct_t.y();
    //     odometry.pose.pose.position.z = correct_t.z();
    //     odometry.pose.pose.orientation.x = correct_q.x();
    //     odometry.pose.pose.orientation.y = correct_q.y();
    //     odometry.pose.pose.orientation.z = correct_q.z();
    //     odometry.pose.pose.orientation.w = correct_q.w();

    //     pose_stamped.pose = odometry.pose.pose;
    //     relo_path.header = header;
    //     relo_path.header.frame_id = "world";
    //     relo_path.poses.push_back(pose_stamped);
    //     pub_relo_path.publish(relo_path);

    //     // write result to file
    //     ofstream foutC(VINS_RESULT_PATH, ios::app);
    //     foutC.setf(ios::fixed, ios::floatfield);
    //     foutC.precision(0);
    //     foutC << header.stamp.toSec() * 1e9 << ",";
    //     foutC.precision(5);
    //     foutC << estimator.Ps[WINDOW_SIZE].x() << ","
    //           << estimator.Ps[WINDOW_SIZE].y() << ","
    //           << estimator.Ps[WINDOW_SIZE].z() << ","
    //           << tmp_Q.w() << ","
    //           << tmp_Q.x() << ","
    //           << tmp_Q.y() << ","
    //           << tmp_Q.z() << ","
    //           << estimator.Vs[WINDOW_SIZE].x() << ","
    //           << estimator.Vs[WINDOW_SIZE].y() << ","
    //           << estimator.Vs[WINDOW_SIZE].z() << "," << endl;
    //     foutC.close();
    // }
}
