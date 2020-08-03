#include "../include/visualizor.h"
#include "../include/util/config.h"

ros::Publisher pub_odometry;
ros::Publisher pub_path, pub_pose;
// ros::Publisher pub_cloud, pub_map;
ros::Publisher pub_key_poses;
// ros::Publisher pub_ref_pose, pub_cur_pose;
// ros::Publisher pub_key;

nav_msgs::Path path;

void registerPub(ros::NodeHandle &n)
{
    pub_path     = n.advertise<nav_msgs::Path>("path", 1000);
    pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pub_key_poses = n.advertise<visualization_msgs::Marker>("key_frames", 1000);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header) 
{
    if (estimator.initial_)
    {
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.header.frame_id = "world";
        odometry.child_frame_id  = "world";

        Quaterniond tmp_Q;
        tmp_Q = estimator.RS_[FEN_WINDOW_SIZE];
        odometry.pose.pose.position.x = estimator.PS_[FEN_WINDOW_SIZE].x();
        odometry.pose.pose.position.y = estimator.PS_[FEN_WINDOW_SIZE].y();
        odometry.pose.pose.position.z = estimator.PS_[FEN_WINDOW_SIZE].z();

        odometry.pose.pose.orientation.x = tmp_Q.x();
        odometry.pose.pose.orientation.y = tmp_Q.y();
        odometry.pose.pose.orientation.z = tmp_Q.z();
        odometry.pose.pose.orientation.w = tmp_Q.w();
        
        odometry.twist.twist.linear.x = estimator.VS_[FEN_WINDOW_SIZE].x();
        odometry.twist.twist.linear.y = estimator.VS_[FEN_WINDOW_SIZE].y();
        odometry.twist.twist.linear.z = estimator.VS_[FEN_WINDOW_SIZE].z();
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = header;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odometry.pose.pose;
        
        path.header = header;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);
    }
}

void pubKeyFrame(const Estimator& estimator, const std_msgs::Header& header) {
    if (!estimator.initial_)
        return;
    
    visualization_msgs::Marker key_poses;
    key_poses.header = header;
    key_poses.header.frame_id = "world";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.05;
    key_poses.scale.y = 0.05;
    key_poses.scale.z = 0.05;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    for (int i = 0; i <= FEN_WINDOW_SIZE; i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d correct_pose;
        correct_pose = estimator.PS_[i];

        pose_marker.x = correct_pose.x();
        pose_marker.y = correct_pose.y();
        pose_marker.z = correct_pose.z();
        key_poses.points.push_back(pose_marker);
    }
    pub_key_poses.publish(key_poses);
}
