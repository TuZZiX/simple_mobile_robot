//
// Created by shipei on 6/23/17.
//

#include <BetterRobotCommander.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "square_move"); //name this node
    ros::NodeHandle nh;
    BetterRobotCommander robot(nh, "path_vel");
    double length = 3;
    if (argc >= 1) {
        length = atof(argv[1]);
    } else {
        std::cout << "Wrong number of parameters! Use default parameter length = " << length << std::endl
                  << "Usage: rosrun simple_mobile_robot square_move [length]" << std::endl
                  << "      [length] is the side length of the square path in meter, if you want the robot to move counter clockwise, specify a negative number"
                  << std::endl
                  << "Example: rosrun simple_mobile_robot square_move 3 # run a clockwise square path of length = 3"
                  << std::endl;
    }
    double direction = -M_PI / 2; // clockwise
    if (length < 0) {
        length = -length;
        direction = -direction; // switch from clockwise to counter clockwise
    }
    ROS_INFO("move to start location");
    robot.move(length / 2);
    robot.waitForFinsh();
    ROS_INFO("Start a square path move with side length = %f, in %s", length,
             (direction < 0 ? "clockwise" : "counter clockwise"));
    int counter = 0;
    while (ros::ok()) {
        if (counter % 4 == 0) {
            ROS_INFO("Loop %d", (counter / 4) + 1);  // count from 1, more friendly
        }
        ROS_INFO("turn %s", (direction < 0 ? "right" : "left"));
        robot.turn(direction);
        robot.waitForFinsh();
        ROS_INFO("move ahead");
        robot.move(length);
        robot.waitForFinsh();
        counter++;
    }
    return 0;
}