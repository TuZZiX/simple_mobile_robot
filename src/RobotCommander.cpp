//
// Created by tianshipei on 2/14/16.
//

#include <RobotCommander.h>

RobotCommander::RobotCommander(ros::NodeHandle &nodeHandle, std::string topic) : nh(nodeHandle) {
    sample_dt = 0.01;
    speed = 1.0; // 1m/s speed command
    yaw_rate = 0.5; //0.5 rad/sec yaw rate command
    twist_commander = nh.advertise<geometry_msgs::Twist>(topic, 10);
}

void RobotCommander::stop() {
    geometry_msgs::Twist twist_cmd;
    twist_commander.publish(twist_cmd);
}

void RobotCommander::turn(double rad) {
    rad = minSpin(rad);
    double time = fabs(rad) / yaw_rate;
    double timer = 0.0;
    ros::Rate loop_timer(1 / sample_dt);
    geometry_msgs::Twist twist_cmd;
    if (rad > 0.0) {
        twist_cmd.angular.z = yaw_rate;
    } else if (rad < 0.0) {
        twist_cmd.angular.z = -1.0 * yaw_rate;
    }
    while (timer < time) {
        twist_commander.publish(twist_cmd);
        timer += sample_dt;
        loop_timer.sleep();
        ros::spinOnce();
    }
    stop();
}

void RobotCommander::spin(int direction) {
    ros::Rate loop_timer(1 / sample_dt);
    geometry_msgs::Twist twist_cmd;
    switch (direction) {
        case NONE:
            break;
        case LEFT:
            twist_cmd.angular.z = yaw_rate;
            break;
        case RIGHT:
            twist_cmd.angular.z = -1.0 * yaw_rate;
            break;
        default:
            break;
    }
    twist_commander.publish(twist_cmd);
}

void RobotCommander::move(int direction, double distance) {
    double timer = 0.0;
    ros::Rate loop_timer(1 / sample_dt);
    geometry_msgs::Twist twist_cmd;
    double time = fabs(distance) / speed;
    switch (direction) {
        case NONE:
            break;
        case FORWARD:
            twist_cmd.linear.x = speed;
            break;
        case BACKWARD:
            twist_cmd.linear.x = -1.0 * speed;
            break;
        default:
            break;
    }
    while (timer < time) {
        twist_commander.publish(twist_cmd);
        timer += sample_dt;
        loop_timer.sleep();
        ros::spinOnce();
    }
    stop();
}

void RobotCommander::move(double distance) {
    if (distance >= 0) {
        move(FORWARD, distance);
    } else {
        move(BACKWARD,1 * distance);
    }
}

void RobotCommander::go(int direction) {
    geometry_msgs::Twist twist_cmd;
    switch (direction) {
        case NONE:
            break;
        case FORWARD:
            twist_cmd.linear.x = speed;
            break;
        case BACKWARD:
            twist_cmd.linear.x = -1.0 * speed;
            break;
        default:
            break;
    }
    twist_commander.publish(twist_cmd);
}

//a function to consider periodicity and find min delta angle
double RobotCommander::minSpin(double spin_angle) {
    while (spin_angle >= M_PI) {
        spin_angle= 2.0 * M_PI;
    }
    while (spin_angle <M_PI) {
        spin_angle += 2.0 * M_PI;
    }
    return spin_angle;
}
