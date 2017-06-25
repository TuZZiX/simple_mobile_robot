#include "DesStatePublisher.h"

bool is_init_orien = false;
geometry_msgs::Pose g_current_pose;

// a useful conversion function: from quaternion to yaw
double quat2ang(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion ang2quat(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void odomCallback(const nav_msgs::Odometry &odom_msg) {
    if (!is_init_orien) {
        is_init_orien = true;
        g_current_pose = odom_msg.pose.pose;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_state_publisher_main");
    ros::NodeHandle nh;
    ros::Subscriber odom_subscriber = nh.subscribe("odom", 1, odomCallback);
    //instantiate a desired-state publisher object
    DesStatePublisher desStatePublisher(nh);
    //dt is set in header file pub_des_state.h    
    ros::Rate looprate(1 / dt); //timer for fixed publication rate
    ROS_INFO("Waiting for odometery message to start...");
    while (!is_init_orien) {
        ros::spinOnce();    //wait for odom callback
    }
    ROS_INFO("got odometery with x = %f, y = %f, th = %f", g_current_pose.position.x, g_current_pose.position.y,
             quat2ang(g_current_pose.orientation));
    desStatePublisher.set_init_pose(g_current_pose.position.x, g_current_pose.position.y,
                                    quat2ang(g_current_pose.orientation)); //x=0, y=0, psi=0
    //put some points in the path queue--hard coded here
    if (argc > 1 && (strcmp(argv[1], "test") == 0)) {
        desStatePublisher.append_path_queue(1, 0.0, 0.0);
    } else if (argc > 1 && (strcmp(argv[1], "square") == 0)) {
        double circle_length = atof(argv[2]);
        desStatePublisher.append_path_queue(circle_length, 0.0, 0.0);
        desStatePublisher.append_path_queue(circle_length, circle_length, -M_PI / 2);
        desStatePublisher.append_path_queue(-1 * circle_length, circle_length, -M_PI);
        desStatePublisher.append_path_queue(-1 * circle_length, -1 * circle_length, M_PI / 2);
        desStatePublisher.append_path_queue(circle_length, -1 * circle_length, 0.0);
        desStatePublisher.append_path_queue(circle_length, circle_length, -M_PI / 2);
        desStatePublisher.append_path_queue(0.0, circle_length, 0.0);
    } else {
        desStatePublisher.append_path_queue(0.0, 0.0, 0.0);
    }
    // main loop; publish a desired state every iteration
    while (ros::ok()) {
        desStatePublisher.pub_next_state();
        ros::spinOnce();
        looprate.sleep(); //sleep for defined sample period, then do loop again
    }
}

