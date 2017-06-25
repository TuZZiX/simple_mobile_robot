//
// Created by shipei on 6/25/17.
//

#ifndef BETTER_ROBOT_COMMANDER_H
#define BETTER_ROBOT_COMMANDER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>

#define NONE        0
#define FORWARD     1
#define BACKWARD    2
#define RIGHT       3
#define LEFT        4


class BetterRobotCommander {
public:
    BetterRobotCommander(ros::NodeHandle &nodeHandle, std::string topic = "cmd_vel");

    // stop the r
    void stop();

    // keep spinning to one direction without stop
    void spin(int direction);

    // make a turn
    void turn(double rad);

    // move forward for backward with distance in m, negative means backward
    void move(double distance);

    // keep going to one direction without stop
    void go(int direction);

    bool finished() {
        return move_dir == NONE && spin_dir == NONE && current_speed == 0 && current_spin_rate == 0;
    }

    void waitForFinsh() {
        ros::Duration time(dt);
        while (!finished()) {
            time.sleep();
        }
    }

    // unify angle within [-180, 180)
    double minSpin(double spin_angle) {
        while (spin_angle >= M_PI) {
            spin_angle -= 2.0 * M_PI;
        }
        while (spin_angle < -M_PI) {
            spin_angle += 2.0 * M_PI;
        }
        return spin_angle;
    }

    void setMaxSpeed(double max_speed) { this->max_speed = max_speed; }

    void setMaxSpinRate(double max_spin_rate) { this->max_spin_rate = max_spin_rate; }

    void setSpin_acc(double spin_acc) { this->spin_acc = spin_acc; }

    void setSpeed_acc(double speed_acc) { this->speed_acc = speed_acc; }

private:
    ros::NodeHandle nh;
    ros::Publisher twist_commander;
    ros::Timer controller_timer;
    ros::AsyncSpinner spinner;

    double dt;

    double max_spin_rate;
    double max_speed;
    double spin_acc;
    double speed_acc;

    double current_speed;
    double current_spin_rate;
    double current_distance;
    double current_spin;

    double target_distance;
    double target_spin;

    int move_dir;
    int spin_dir;

    void updateCallback(const ros::TimerEvent &event);
};


#endif //BETTER_ROBOT_COMMANDER_H
