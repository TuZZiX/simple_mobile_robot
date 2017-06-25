//
// Created by shipei on 6/25/17.
//

#include "BetterRobotCommander.h"

BetterRobotCommander::BetterRobotCommander(ros::NodeHandle &nodeHandle, std::string topic) : nh(nodeHandle), spinner(1) {
    dt = 0.01;
    max_speed = 1.0; // 1m/s speed command
    max_spin_rate = 0.5; //0.5 rad/sec yaw rate command
    speed_acc = 1.5;
    spin_acc = 1.0;
    twist_commander = nh.advertise<geometry_msgs::Twist>(topic, 10);
    controller_timer = nh.createTimer(ros::Duration(dt), &BetterRobotCommander::updateCallback, this);
    spinner.start();
}

void BetterRobotCommander::stop() {
    move_dir = NONE;
    spin_dir = NONE;
    current_speed = 0;
    current_spin_rate = 0;
}

void BetterRobotCommander::go(int direction) {
    current_distance = 0;
    move_dir = direction;
    target_distance = INFINITY;
}

void BetterRobotCommander::move(double distance) {
    current_distance = 0;
    if (distance >= 0) {
        move_dir = FORWARD;
    } else {
        move_dir = BACKWARD;
    }
    target_distance = distance;
}

void BetterRobotCommander::spin(int direction) {
    current_spin = 0;
    spin_dir = direction;
    target_spin = INFINITY;
}

void BetterRobotCommander::turn(double rad) {
    current_spin = 0;
    if (rad > 0.0) {
        spin_dir = LEFT;
    } else if (rad < 0.0) {
        spin_dir = RIGHT;
    }
    target_spin = rad / 1.1;
}

void BetterRobotCommander::updateCallback(const ros::TimerEvent &event) {
    double spin_to_stop = (current_spin_rate * current_spin_rate / spin_acc) / 2;
    if (fabs(target_spin - current_spin) < spin_to_stop) {
        spin_dir = NONE;
    }
    double distance_to_stop = (current_speed * current_speed / speed_acc) / 2;
    if (fabs(target_distance - current_distance) < distance_to_stop) {
        move_dir = NONE;
    }
    if (spin_dir == LEFT) {
        if (current_spin_rate < max_spin_rate) {
            current_spin_rate += spin_acc * dt;
        } else {
            current_spin_rate = max_spin_rate;
        }
    } else if (spin_dir == RIGHT) {
        if (current_spin_rate > -max_spin_rate) {
            current_spin_rate -= spin_acc * dt;
        } else {
            current_spin_rate = -max_spin_rate;
        }
    } else if (spin_dir == NONE) {
        // use stop deceleration, infinite jerk
        if (current_spin_rate > spin_acc * dt) {
            current_spin_rate -= spin_acc * dt;
        } else if (current_spin_rate > 0) {
            current_spin_rate = 0;
        } else if (current_spin_rate < -spin_acc * dt) {
            current_spin_rate -= -spin_acc * dt;
        } else {
            current_spin_rate = 0;
        }
    }
    current_spin += current_spin_rate * dt;
    // same for forward
    if (move_dir == FORWARD) {
        if (current_speed < max_speed) {
            current_speed += speed_acc * dt;
        } else {
            current_speed = max_speed;
        }
    } else if (move_dir == BACKWARD) {
        if (current_speed > -max_speed) {
            current_speed += speed_acc * dt;
        } else {
            current_speed = -max_speed;
        }
    } else if (move_dir == NONE) {
        // use stop deceleration, infinite jerk
        if (current_speed > speed_acc * dt) {
            current_speed -= speed_acc * dt;
        } else if (current_speed > 0) {
            current_speed = 0;
        } else if (current_speed < -speed_acc * dt) {
            current_speed -= -speed_acc * dt;
        } else {
            current_speed = 0;
        }
    }
    current_distance += current_speed * dt;
    geometry_msgs::Twist twist;
    twist.angular.z = current_spin_rate;
    twist.linear.x = current_speed;
    twist_commander.publish(twist);
    if (spin_dir == NONE && current_spin_rate == 0) {
        current_spin = 0;
    }
    if (move_dir == NONE && current_speed == 0) {
        current_distance = 0;
    }
}
