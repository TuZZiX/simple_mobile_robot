#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//node to receive desired states and republish the twist as cmd_vel commands
ros::Publisher republisher;

//simply copy the desired twist and republish it to cmd_vel
void desStateCallback(const nav_msgs::Odometry &des_state) {
    geometry_msgs::Twist twist = des_state.twist.twist;
    republisher.publish(twist);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_forwarder");
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    republisher = n.advertise<geometry_msgs::Twist>("/path_vel", 10);
    ros::Subscriber des_state_subscriber = n.subscribe("/des_state", 10, desStateCallback);
    ros::spin();
}

