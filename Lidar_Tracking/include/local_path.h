#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>
#include<queue>
#include<tf/tf.h>
#include <tf/transform_datatypes.h>

#include<std_msgs/Int32.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>

#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/OccupancyGrid.h>

#include<geometry_msgs/Point.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include<visualization_msgs/MarkerArray.h>

#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>


#include "pure_pursuit.h"
#include "ros_data.h"
#include "PID.h"




class local_path : public pure_pursuit, ros_data, PIDController{
public:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

   
    local_path(double kp, double ki, double kd);
    double rate = 10;
    int prev_temp_num;
    int num;
    double target_distance;


    double index;
    
    
  

    std_msgs::Float64 Motor_cmd;
    std_msgs::Float64 Steering_cmd;

    //Function
    void path_tracking(nav_msgs::Path global_path);


    void process();
};
