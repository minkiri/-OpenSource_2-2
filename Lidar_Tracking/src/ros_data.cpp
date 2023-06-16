#include "ros_data.h"


ros_data::ros_data() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    //publish
    cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/high_level/ackermann_cmd_mux/input/nav_0", 1);
    //topic for wecar 2.0

    //subscibe

    lidar_waypt_sub = nh.subscribe("/center_points", 10, &ros_data::way_pt_callback, this);
    platoon_flag_sub = nh.subscribe("/platoon_go", 10, &ros_data::platoon_flag_callback, this);
    charuco_pose_sub = nh.subscribe("/charuco_pose", 10, &ros_data::charuco_pose_callback, this);
    front_current_vel_sub = nh.subscribe("/front_distance",10,&ros_data::front_current_vel_callback,this);
}

void ros_data::charuco_pose_callback(const geometry_msgs::PoseStamped &msg)
{
    ROS_INFO("Pose callback");
    charuco_pose_ = msg;
    charuco_orientation_ = charuco_pose_.pose.orientation;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(charuco_orientation_, quat);
    tf::Matrix3x3 mat(quat);
    cv::Mat rotationMatrix = cv::Mat::eye(3, 3, CV_64F);
    rotationMatrix.at<double>(0, 0) = 0.0;
    rotationMatrix.at<double>(0, 1) = 1.0;
    rotationMatrix.at<double>(1, 1) = 0.0;
    rotationMatrix.at<double>(1, 2) = -1.0;
    rotationMatrix.at<double>(2, 2) = 1.0;
    mat.getRPY(roll_, pitch_, yaw_);
    double correctedYaw = yaw_ + calculateYawCorrection(rotationMatrix);
    std::cout<< "correctedYaw : " <<correctedYaw<<std::endl;
    //std::cout<<charuco_pose_.pose.position.x<<std::endl;
}

//void ros_data::way_pt_callback(const sensor_msgs::PointCloud2 &msg) {
//    sensor_msgs::convertPointCloud2ToPointCloud(msg, lidar_waypoint);
//}
void ros_data::way_pt_callback(const sensor_msgs::PointCloud &msg) {
    //ROS_INFO("Pose callback");
lidar_waypoint = msg;
}

void ros_data::platoon_flag_callback(const std_msgs::Bool &msg) {
    platoon_state = msg.data;
}

void ros_data::front_current_vel_callback(const std_msgs::Float32 &msg) {
    front_vel_ = msg.data;
}