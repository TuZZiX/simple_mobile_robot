//
// Created by shipei on 6/23/17.
//

#include <RobotCommander.h>



int main(int argc, char **argv) {
    ros::init(argc, argv, "path_move"); //name this node
    ros::NodeHandle nh;
    RobotCommander robot(nh, "path_vel");
    robot.move(1.5);
    ROS_INFO("move ahead");
    while (ros::ok()) {
        ROS_INFO("turn right");
        robot.turn(-M_PI/2 + 0.1);
        ros::Duration(0.5).sleep();
        ROS_INFO("move ahead");
        robot.move(3);
    }
    return 0;
}