#include<ros/ros.h>
#include<math.h>
#include<cmath>
#include<vector>
#include<queue>
#include<tf/tf.h>
#include<visualization_msgs/MarkerArray.h>
#include<std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>

#include"ros_data.h"

class pure_pursuit{
    //Subscriber
   
    double max_lfd, min_lfd, VL, L;
    double  back_x, back_y;

public:
    pure_pursuit();
    bool is_track_look_forward_point;
    //Publisher
    ros::Publisher marker_lfd;
    double obs_dist_;


    //Function

    void lfd_visualiztion(geometry_msgs::Pose index);
    double track_steering_angle(sensor_msgs::PointCloud way_pt);
    double marker_steering_angle(geometry_msgs::PoseStamped charuco_waypt);
};
