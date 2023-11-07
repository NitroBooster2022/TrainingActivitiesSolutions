#include <ros/ros.h>
#include "utils/IMU.h"
#include "utils/localisation.h"
#include <std_msgs/String.h>

class Controller {

private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub,gps_sub;
    ros::Publisher cmd_vel_pub;
    double P, D;
    double A_left, M_left, K_left;
    double last_error;
    double x, y;
    double destination_orientation;
    int orientation;
    std::vector<std::array<std::array<int, 2>, 2>> rotationMatrices;

public:
    Controller() : nh() {
        if (!nh.getParam("/controller/A_left",A_left)){
            ROS_INFO("Can't read /controller/A_left");
        }
        if (!nh.getParam("/controller/M_left",M_left)){
            ROS_INFO("Can't read /controller/M_left");
        }
        if (!nh.getParam("/controller/K_left",K_left)){
            ROS_INFO("Can't read /controller/K_left");
        }
        P = 1.57;
        D = 1;
        imu_sub = nh.subscribe("/automobile/IMU", 1, &Controller::IMU_Callback, this);
        gps_sub = nh.subscribe("/automobile/localisation", 1, &Controller::GPS_Callback, this);
        cmd_vel_pub = nh.advertise<std_msgs::String>("/automobile/command", 1);
        utils::IMU i = *(ros::topic::waitForMessage<utils::IMU>("/automobile/IMU", nh));
        destination_orientation = i.yaw+M_PI/2;
        if (destination_orientation>M_PI) {
            destination_orientation -= 2*M_PI;
        }
        double orientations[] = {std::abs(i.yaw), std::abs(i.yaw - 1.5708), std::abs(i.yaw - 3.14159), std::abs(i.yaw - 4.71239), std::abs(i.yaw - 6.28319)};
        int minOrientationIndex = 0;
        for (int i = 1; i < 5; ++i) {
            if (orientations[i] < orientations[minOrientationIndex]) {
                minOrientationIndex = i;
            }
        }
        orientation = minOrientationIndex % 4;
        rotationMatrices.push_back({{{1, 0}, {0, 1}}});   // East
        rotationMatrices.push_back({{{0, -1}, {1, 0}}});  // North
        rotationMatrices.push_back({{{-1, 0}, {0, -1}}}); // West
        rotationMatrices.push_back({{{0, 1}, {-1, 0}}});  // South
    }

    void IMU_Callback(const utils::IMU& msg) {
        double current_orientation = msg.yaw;
        std::vector<double> poses = {x, y};
        std::array<std::array<int, 2>, 2> rotationMatrix = rotationMatrices[orientation];
        double x_trans = poses[0] * rotationMatrix[0][0] + poses[1] * rotationMatrix[1][0];
        double y_trans = poses[0] * rotationMatrix[0][1] + poses[1] * rotationMatrix[1][1];
        double desiredY = A_left*exp(M_left*x+K_left);
        double error = -abs(y_trans - desiredY);
        bool arrived = abs(current_orientation-destination_orientation) <= 0.15 | abs(current_orientation-destination_orientation) >= 6.13;
        if (arrived) {
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

    void GPS_Callback(const utils::localisation& msg) {
        x = msg.posA;
        y = msg.posB;
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