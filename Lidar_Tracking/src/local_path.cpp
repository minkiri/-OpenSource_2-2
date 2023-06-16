#include "local_path.h"

// semi fianl test 했던 파일임
local_path::local_path(double kp, double ki, double kd)
        : PIDController(kp, ki, kd), nh(""), pnh("~") {

    pnh.param<double>("target_distance", target_distance, 1.0);


}

void local_path::process() {

    if (is_track_look_forward_point == true) {
        //ROS_INFO("Doing Platooning");
        PIDController pidController(0.2, 0.1, 0.05);
        double dt = 0.01;
//        double target_distance = 0.3;

        double current_distance = obs_dist_;
        std::cout<<"OBS DIST : "<<obs_dist_ <<std::endl;
        double control_output = pidController.calculateOutput(target_distance, current_distance, dt);
        control_output *= front_vel_;
        double speed = front_vel_-0.4;
        if(speed >0.9)
            speed = 0.9;
        if(platoon_state == false)
        {
            motor_msg.drive.speed = 0.0;
        } else {
            motor_msg.drive.speed = speed;
        }
        if(front_vel_<0.65)
        {
            motor_msg.drive.speed = 0.0;
            ROS_INFO("AEB!!!!!!");
        }
        //motor_msg.drive.steering_angle = 1.0 * marker_steering_angle(charuco_pose_);

        //Lidar Based Platooning
        motor_msg.drive.steering_angle = 0.8 * track_steering_angle(lidar_waypoint);
    } else{
        motor_msg.drive.speed = 0;
        motor_msg.drive.steering_angle = 0;
        ROS_INFO("No Tracking Car");

    }
    cmd_pub.publish(motor_msg);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Marker_Tracking");

    ros_data data;
    local_path path(0, 0, 0);
//    data.flag = false;

    ros::Rate loop_rate(path.rate);
    while (ros::ok()) {
        path.track_steering_angle(data.lidar_waypoint);
        path.process();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
