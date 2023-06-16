#include "pure_pursuit.h"
#include "ros_data.h"

pure_pursuit::pure_pursuit()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
//    pnh.param("L", L, 0.1);
//    pnh.param("VL",VL,1.0);
    pnh.param("L", L, 0.0);
    pnh.param("VL",VL,0.35);
    pnh.param("max_lfd", max_lfd, 1.7);
    pnh.param("min_lfd", min_lfd, 0.3);

    marker_lfd = nh.advertise<visualization_msgs::MarkerArray>("/Look_Forward_Distance",10);
    obs_dist_ = 0.0;
}

void pure_pursuit::lfd_visualiztion(geometry_msgs::Pose index)
{
    visualization_msgs::MarkerArray node_arr;
    visualization_msgs::Marker node1;
    node1.header.frame_id = "map"; // map frame 기준
    node1.header.stamp = ros::Time::now();
    node1.type = visualization_msgs::Marker::SPHERE;
    node1.id = 0;
    node1.action = visualization_msgs::Marker::ADD;
    node1.pose.orientation.w = 1.0;
    node1.pose.position.x = index.position.x; //노드의 x 좌표
    node1.pose.position.y = index.position.y; //노드의 y 좌표 // Points are green
    node1.color.r = 1.0;
    node1.color.a = 1.0;
    node1.scale.x = 0.2;
    node1.scale.y = 0.2;
    node_arr.markers.push_back(node1);

    marker_lfd.publish(node_arr);
}
double pure_pursuit::marker_steering_angle(geometry_msgs::PoseStamped charuco_waypt)
{
    double min_dist = 1;
    double steering = 0;
    double dis= 0;
    double lfd = 0.4;

//    for (int i = 0; i < charuco_waypt.pose.position.s; ++i) {
//
//    }
//    for(int i = 0; i<charuco_waypt.points.size(); i++)
//    {
//        double dx = back_x - charuco_waypt.points.at(i).x;
//        double dy = back_y - charuco_waypt.points.at(i).y;
//
//        double dist = sqrt(dx*dx + dy*dy);
//        if(dist<min_dist)
//        {
//            min_dist = dist;
//            min_index = i;
//        }
//    }

    //lfd = cmd_vel.linear.x/vel_param;
    //lfd=1.6;

    if(lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if(lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    double dx,dy = 0;


        dx = charuco_waypt.pose.position.x;
        dy = charuco_waypt.pose.position.y;
        std::cout<<dx<<std::endl;
        if(dx > 0)
        {
            dis = sqrt(pow(dx,2) + pow(dy,2));
            //std::cout<<"dist : " <<dis <<std::endl;
            double por_dist = dis+0.5;
            if(por_dist>=lfd)
            {
                is_track_look_forward_point = true;
            }
            else
                ROS_INFO("por_dist < LFD");
        }
        else
            ROS_INFO("dx is not bigger than 0");



    double theta = atan2(dy,dx);

    if(is_track_look_forward_point == true)
    {
        double eta = atan2((2*VL*sin(theta)),lfd);
        steering = eta;
    }
    else
    {
        ROS_INFO("no found track forwad point");
    }

    return steering;
}
double pure_pursuit::track_steering_angle(sensor_msgs::PointCloud way_pt) {
    double min_dist = 1;
    double min_index = 0;
    double steering = 0;
    double dis = 0;
    double lfd = 0.5;
    //ROS_INFO("lfd: %f", lfd);


    for(int i = 0; i<way_pt.points.size(); i++)
    {
        double dx = way_pt.points.at(i).x;
        double dy = way_pt.points.at(i).y;

        double dist = sqrt(dx*dx + dy*dy);
        if(dist<min_dist)
        {
            min_dist = dist;
            min_index = i;
        }
    }

    //lfd = cmd_vel.linear.x/vel_param;
    //lfd=1.6;

    if(lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if(lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    double dx,dy = 0;
    for(int i = 0; i<way_pt.points.size(); i++)
    {
        dx = way_pt.points.at(i).x+0.5;
        dy = way_pt.points.at(i).y;
        //std::cout<<"dx  , dy : " <<dx<<" "<<dy<<std::endl;
        if(dx > 0)
        {

            dis = sqrt(pow(dx,2) + pow(dy,2));
            //std::cout<<"dist : " <<dis <<std::endl;
            obs_dist_ = dis;

            if(dis>=lfd)
            {
                is_track_look_forward_point = true;
                break;
            }
            else
                ROS_INFO("dis < LFD");
        }
        else
           ROS_INFO("dx is not bigger than 0");
    }


    double theta = atan2(dy,dx);

    if(is_track_look_forward_point == true)
    {
        double eta = atan2((2*VL*sin(theta)),lfd);
        steering = eta;
    }
    else
    {
        ROS_INFO("no found track forwad point");
    }

    return steering;
}


// double pure_pursuit::steering_angle(geometry_msgs::PoseWithCovariance pose, nav_msgs::Path tracking_path, double vehicle_yaw, double velocity)
// {
//     geometry_msgs::Pose index;

//     is_look_foward_point = false;

//     double back_x = pose.pose.position.x - L*cos(vehicle_yaw);
//     double back_y = pose.pose.position.y - L*sin(vehicle_yaw);

//     double dis = 0;

//     double lfd;
//     double max_lfd = this->max_lfd;
//     double min_lfd = this->min_lfd;
//     double rotated_x = 0;
//     double rotated_y = 0;
// //
//    //lfd = 1.0;
//     lfd = velocity/6.5;
// //   if(velocity<4)
// //   {
// //       lfd = 0.4;
// //   }
// //   else if(velocity>4&&velocity<8)
// //   {
// //       lfd = 0.6;
// //   }
// //   else{
// //       lfd = 1.0;
// //   }

//    //lfd = velocity/6.0;
//    //lfd = velocity/6.0;
//    ROS_INFO(" %lf \n",lfd);

//     if( avoid == true){
//        lfd = 0.5;
//      //ROS_INFO("Lfd changed: %lf\n",lfd);
//     }
//     if(lfd < min_lfd)
//     {
//         lfd = min_lfd;
//     }
//     else if(lfd > max_lfd)
//     {
//         lfd = max_lfd;
//     }
//     for(int i = 0; i<tracking_path.poses.size(); i++)
//     {
//         double dx = tracking_path.poses.at(i).pose.position.x - back_x;
//         double dy = tracking_path.poses.at(i).pose.position.y - back_y;

//         rotated_x = cos(vehicle_yaw)*dx + sin(vehicle_yaw)*dy;
//         rotated_y = -sin(vehicle_yaw)*dx + cos(vehicle_yaw)*dy;

//         if(rotated_x > 0)
//         {
//             dis = sqrt(pow(rotated_x,2) + pow(rotated_y,2));
//             //ROS_INFO("dis: %lf, lfd: %lf", dis, lfd);
//             if(dis>=lfd)
//             {
//                 index.position.x = tracking_path.poses.at(i).pose.position.x;
//                 index.position.y = tracking_path.poses.at(i).pose.position.y;
//                 is_look_foward_point = true;
//                 break;
//             }
//             else {
//                 //ROS_ERROR("lfd is bigger than dis");
//             }
//         }
//         else {
//             //ROS_ERROR("rotated_x is minus");
//         }
//     }

//     double theta = atan2(rotated_y,rotated_x);
//     double steering = 0;
//     if(is_look_foward_point == true)
//     {
//         //ROS_INFO("LFD founded");
//         double eta = atan2((2*VL*sin(theta)),lfd)*180/M_PI;
//             steering = eta;
//         lfd_visualiztion(index);
//     }
//     else
//     {
//         ROS_INFO("no found forwad point");
//     }

//     return steering;
// }