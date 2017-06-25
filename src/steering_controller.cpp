//NlSteering.cpp:
//wsn, Feb 2016
//subscribes to topics for desired state and actual state
// invokes a nonlinear steering algorithm to command speed and spin on cmd_vel topic

// this header incorporates all the necessary #include files and defines the class "SteeringController"
#include <SteeringController.h>


SteeringController::SteeringController(ros::NodeHandle &nodehandle) : nh(nodehandle) { // constructor
    ROS_INFO("in class constructor of SteeringController");
    UPDATE_RATE = 100.0; // choose the update rate
    K_PSI = 1.0; // control gains for steering
    K_LAT_ERR_THRESH = 1.0;
    K_TRIP_DIST = 1.0;
    MAX_SPEED = 1.0; // m/sec; tune this
    MAX_SPIN_RATE = 0.8; // rad/sec; tune this
    K_PHI = 1.0;

    nh.param("UPDATE_RATE", UPDATE_RATE, UPDATE_RATE);
    nh.param("K_PSI", K_PSI, K_PSI);
    nh.param("K_LAT_ERR_THRESH", K_LAT_ERR_THRESH, K_LAT_ERR_THRESH);
    nh.param("MAX_SPEED", MAX_SPEED, MAX_SPEED);
    nh.param("MAX_SPIN_RATE", MAX_SPIN_RATE, MAX_SPIN_RATE);
    nh.param("K_TRIP_DIST", K_TRIP_DIST, K_TRIP_DIST);
    nh.param("K_PHI", K_PHI, K_PHI);

    des_state_subscriber_ = nh.subscribe("/des_state", 1, &SteeringController::desStateCallback, this);
    odom_subscriber_ = nh.subscribe("/odom", 1, &SteeringController::odomCallback, this); //subscribe to odom messages
    cmd_publisher_ = nh.advertise<geometry_msgs::Twist>("path_vel", 1, true); // commands the robot!

    state_psi_ = 1000.0; // put in impossible value for heading; 
    //test this value to make sure we have received a viable state message
    ROS_INFO("waiting for valid state message...");
    while (state_psi_ > 500.0) {
        ros::Duration(0.5).sleep(); // sleep for half a second
        std::cout << ".";
        ros::spinOnce();
    }
    ROS_INFO("constructor: got a state message");

    //initialize desired state;  can be changed dynamically by publishing to topic /desState
    des_state_speed_ = MAX_SPEED; //can make dynamic via des_state_rcvd.twist.twist.linear.x;
    des_state_omega_ = 0.0; //des_state_rcvd.twist.twist.angular.z;

    // hard code a simple path: the world x axis
    des_state_x_ = 0.0;
    des_state_y_ = 0.0;
    des_state_psi_ = 0.0;

    sleep_timer.fromSec(1 / UPDATE_RATE);
}

void SteeringController::odomCallback(const nav_msgs::Odometry &odom_rcvd) {
    state_speed_ = odom_rcvd.twist.twist.linear.x;
    state_omega_ = odom_rcvd.twist.twist.angular.z;
    state_x_ = odom_rcvd.pose.pose.position.x;
    state_y_ = odom_rcvd.pose.pose.position.y;
    state_quat_ = odom_rcvd.pose.pose.orientation;
    state_psi_ = convertPlanarQuat2Phi(state_quat_); // cheap conversion from quaternion to heading for planar motion
}

//use this if a desired state is being published
void SteeringController::desStateCallback(const nav_msgs::Odometry &des_state_rcvd) {
    // copy some of the components of the received message into member vars
    // we care about speed and spin, as well as position estimates x,y and heading
    des_state_speed_ = des_state_rcvd.twist.twist.linear.x;
    des_state_omega_ = des_state_rcvd.twist.twist.angular.z;
    des_state_x_ = des_state_rcvd.pose.pose.position.x;
    des_state_y_ = des_state_rcvd.pose.pose.position.y;
    des_state_pose_ = des_state_rcvd.pose.pose;
    des_state_quat_ = des_state_rcvd.pose.pose.orientation;
    //Convert quaternion to simple heading
    des_state_psi_ = convertPlanarQuat2Phi(des_state_quat_);
}

//utility fnc to compute min delta angle, accodom_subscriber_ = nh.subscribe("/odom", 1, &SteeringController::odomCallback, this); //subscribe to odom messagesounting for periodicity
double SteeringController::minSpin(double angle) {
    while (angle >= M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}


// saturation function
double SteeringController::sat(double x, double max = 1.0) {
    if (x > max) {
        return max;
    }
    if (x < -max) {
        return -max;
    }
    return x;
}

//some conversion utilities:
double SteeringController::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}


// HERE IS THE STEERING ALGORITHM: USE DESIRED AND ACTUAL STATE TO COMPUTE AND PUBLISH CMD_VEL
void SteeringController::NlSteering() {
    // have access to: des_state_vel_, des_state_omega_, des_state_x_, des_state_y_,
    //  des_state_phi_ and corresponding robot state values
    double tx = cos(des_state_psi_); // [tx,ty] is tangent of desired path
    double ty = sin(des_state_psi_);
    double nx = -ty; //components [nx, ny] of normal to path, points to left of desired heading
    double ny = tx;

    double dx = state_x_ - des_state_x_;  //x-error relative to desired path
    double dy = state_y_ - des_state_y_;  //y-error

    double lateral_err_ = dx * nx + dy * ny; //lateral error is error vector dotted with path normal
    // lateral offset error is positive if robot is to the left of the path
    double trip_dist_err = dx * tx + dy * ty; // progress error: if positive, then we are ahead of schedule
    //heading error: if positive, should rotate -omega to align with desired heading
    double heading_err = minSpin(des_state_psi_ - state_psi_);
    double strategy_psi = minSpin(lateral_err_ + K_PHI * heading_err); //heading command, based on NL algorithm
    double controller_omega = omegaCmdFnc(strategy_psi, state_psi_, des_state_psi_);

    double controller_speed = sat(sat(des_state_speed_, MAX_SPEED) + sat(K_TRIP_DIST * trip_dist_err, MAX_SPEED), MAX_SPEED); // default...should speed up/slow down appropriately
    // send out our speed/spin commands:
    geometry_msgs::Twist twist_cmd_;
    twist_cmd_.linear.x = controller_speed;
    twist_cmd_.angular.z = controller_omega;
    cmd_publisher_.publish(twist_cmd_);

    ros::spinOnce();
    sleep_timer.sleep();
}

double SteeringController::psiStrategy(double offset_err) {
    return -(M_PI / 2) * sat(offset_err / K_LAT_ERR_THRESH);
}

double SteeringController::omegaCmdFnc(double psi_strategy, double psi_state, double psi_path) {
    double omega_cmd = K_PSI * (psi_strategy + psi_path - psi_state); //computed heading command
    omega_cmd = sat(omega_cmd, MAX_SPIN_RATE); //saturate the command at specified limit
    return omega_cmd;
}

int main(int argc, char **argv) {
    // ROS set-ups:
    ros::init(argc, argv, "steering_controller"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
    SteeringController steeringController(nh);

    ROS_INFO("starting steering algorithm");
    while (ros::ok()) {
        // compute and publish twist commands 
        steeringController.NlSteering();
    }
    return 0;
} 

