#ifndef STEERING_CONTROLLER_H_
#define STEERING_CONTROLLER_H_

#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


// define a class, including a constructor, member variables and member functions
class SteeringController
{
public:
    SteeringController(ros::NodeHandle &nodehandle);
    void NlSteering(); // use state and desired state to compute twist command, and publish it
    double psiStrategy(double); //computes strategic heading from lateral path-following error
    double omegaCmdFnc(double psi_strategy, double psi_state, double psi_path);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    double minSpin(double angle);
    double sat(double x, double max);
private:
    ros::NodeHandle nh; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber and publisher
    ros::Subscriber des_state_subscriber_; //publisher for states corresponding to ideal path following
    //ros::Subscriber current_state_subscriber_; //topic to receive estimates of current robot state
    ros::Subscriber odom_subscriber_;
    //initially, get state values from gazebo; need to replace with a sensor-based localization node

    ros::Publisher cmd_publisher_; // sends twist commands to cmd_vel topic
    ros::Duration sleep_timer;

    //state variables, (x,y,psi) and (speed, omega)
    double state_x_;
    double state_y_;
    double state_psi_;
    double state_speed_;
    double state_omega_;

    geometry_msgs::Quaternion state_quat_;

    //state values from desired state; these will get filled in by desStateCallback
    double des_state_x_;
    double des_state_y_;
    double des_state_psi_;
    double des_state_speed_;
    double des_state_omega_;

    geometry_msgs::Quaternion des_state_quat_;
    geometry_msgs::Pose des_state_pose_;

    double UPDATE_RATE; // choose the update rate for steering controller
    double K_PSI; // control gains for steering
    double K_LAT_ERR_THRESH;
    double K_TRIP_DIST;
    double K_PHI;
// dynamic limitations:
    double MAX_SPEED; // m/sec; tune this
    double MAX_SPIN_RATE; // rad/sec; tune this

    //void gazeboPoseCallback(const geometry_msgs::Pose& gazebo_pose);
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    void desStateCallback(const nav_msgs::Odometry& des_state_rcvd);
}; // end of class definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
