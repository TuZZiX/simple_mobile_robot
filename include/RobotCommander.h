//
// Created by tianshipei on 2/14/16.
//

#ifndef ROBOT_COMMANDER_H
#define ROBOT_COMMANDER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <string>

#define NONE        0
#define FORWARD     1
#define BACKWARD    2
#define RIGHT       3
#define LEFT        4


class RobotCommander {
public:
    RobotCommander(ros::NodeHandle &nodeHandle, std::string topic = "cmd_vel");
    // stop the r
    void stop();
    // keep spinning to one direction without stop
    void spin(int direction);
    // make a turn
    void turn(double rad);
    // move forward for backward with distance in m
    void move(int direction, double distance);
    // same as move, negative means backward
    void move(double distance);
    // keep going to one direction without stop
    void go(int direction);
    // unify angle within [-180, 180)
    double minSpin(double spin_angle);

    void setSpeed(double speed) { this->speed = speed; }

    void setYawRate(double yaw_rate) { this->yaw_rate = yaw_rate; }

private:
    ros::NodeHandle nh;
    ros::Publisher twist_commander;
    double sample_dt;
    double speed;
    double yaw_rate;
};

#endif //ROBOT_COMMANDER_H
