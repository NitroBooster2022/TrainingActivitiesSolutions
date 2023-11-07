#include <ros/ros.h>
#include "utils/IMU.h"
#include <std_msgs/String.h>

class Controller {

private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Publisher cmd_vel_pub;
    double P, D;
    double target_orientation, tolerance;
    double last_error;

public:
    Controller() : nh() {
        if (!nh.getParam("/controller/target_orientation",target_orientation)){
            ROS_INFO("Can't read /controller/target_orientation");
        }
        target_orientation = target_orientation/180*M_PI;
        if (!nh.getParam("/controller/tolerance",tolerance)){
            ROS_INFO("Can't read /controller/tolerance");
        }
        tolerance = tolerance/180*M_PI;
        P = 1.57;
        D = 0.3;
        imu_sub = nh.subscribe("/automobile/IMU", 1, &Controller::Callback, this);
        cmd_vel_pub = nh.advertise<std_msgs::String>("/automobile/command", 1);
    }

    void Callback(const utils::IMU& msg) {
        double current_orientation = msg.yaw;
        double error = current_orientation - target_orientation;
        if (abs(error)<tolerance) {
            std_msgs::String m;
            m.data = "{\"action\":\"3\",\"steerAngle\":" + std::to_string(0) + "}";
            cmd_vel_pub.publish(m);
            return;
        }
        else {
            double dt = 0.01;
            double derivative = (error - last_error)/dt;
            double steering_angle = P * error + D * derivative;
            const double min_value = -0.4;
            const double max_value = 0.4;
            if (steering_angle < min_value) {
                steering_angle = min_value;
            } else if (steering_angle > max_value) {
                steering_angle = max_value;
            }
            std_msgs::String m1;
            m1.data = "{\"action\":\"1\",\"speed\":" + std::to_string(0.1) + "}";
            std_msgs::String m2;
            m2.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(steering_angle * 180 / M_PI) + "}";
            cmd_vel_pub.publish(m1);
            cmd_vel_pub.publish(m2);
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");
    Controller Controller;
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}