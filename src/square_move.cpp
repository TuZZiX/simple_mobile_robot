//
// Created by shipei on 6/23/17.
//

#include <BetterRobotCommander.h>



int main(int argc, char **argv) {
    ros::init(argc, argv, "square_move"); //name this node
    ros::NodeHandle nh;
    BetterRobotCommander robot(nh, "path_vel");
    ROS_INFO("move to start location");
    robot.move(1.5);
    robot.waitForFinsh();
    ros::Duration(0.5).sleep();
    while (ros::ok()) {
        ROS_INFO("turn right");
        robot.turn(-M_PI/2);
        robot.waitForFinsh();
        ROS_INFO("move ahead");
        robot.move(3);
        robot.waitForFinsh();
    }
    return 0;
}