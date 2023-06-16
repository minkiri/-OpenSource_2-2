#ifndef ROS_DATA_H
#define ROS_DATA_H
#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>
#include<queue>
#include<tf/tf.h>

#include <opencv2/opencv.hpp>

#include<std_msgs/String.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Int16.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Bool.h>

#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/OccupancyGrid.h>

#include<geometry_msgs/Point.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/Polygon.h>

#include<visualization_msgs/MarkerArray.h>

#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>

#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


class ros_data
{
public:
    ros_data();

    double roll_, pitch_, yaw_;
    float front_vel_;
    bool platoon_state;
    
    sensor_msgs::PointCloud object_point;
    sensor_msgs::PointCloud lidar_waypoint;
    geometry_msgs::PoseStamped charuco_pose_;
    geometry_msgs::Quaternion  charuco_orientation_;
    ackermann_msgs::AckermannDriveStamped motor_msg;


    //Publish
    ros::Publisher cmd_pub;
    //Subscriber
    
    ros::Subscriber object_sub;
    ros::Subscriber lidar_waypt_sub;
    ros::Subscriber platoon_flag_sub;
    ros::Subscriber charuco_pose_sub;
    ros::Subscriber front_current_vel_sub;


    //Callback

    void front_current_vel_callback(const std_msgs::Float32 &msg);
    void way_pt_callback(const sensor_msgs::PointCloud &msg);
    void platoon_flag_callback(const std_msgs::Bool &msg);
    void charuco_pose_callback(const geometry_msgs::PoseStamped &msg);
    double calculateYawCorrection(const cv::Mat& rotationMatrix)
    {
        // 로봇 좌표계에서 영상 좌표계로의 회전 각도 계산
        double yawCorrection = atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));

        // 보정된 yaw 값을 반환
        return yawCorrection;
    }
};

#endif // ROS_DATA_H
