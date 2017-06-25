#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <BetterRobotCommander.h>

/** Set alarm if anything is within 0.5m of the front of robot, or too
    close to the sides. */
double MIN_SAFE_DISTANCE = 0.8;
double ROBOT_WIDTH = 0.6;

/** these values to be set within the laser callback */
bool first_pass = true;
bool laser_alarm_l = false;
bool laser_alarm_r = false;
bool brake_enabled = false;
/** Precomputed for speed. */
std::vector<double> angle_cache_sin;
std::vector<double> angle_cache_cos;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_alarm_publisher_l;
ros::Publisher lidar_alarm_publisher_r;


void laserCallback(const sensor_msgs::LaserScan &laser_scan) {
    static ros::NodeHandle nh;
    static BetterRobotCommander robot(nh, "brake_vel");   // temp object for braking the robot
    std_msgs::Bool lidar_alarm_msg;
    int i = laser_scan.ranges.size();
    double angle_min_, angle_increment_, theta_i, x, y, r;
    double robot_radius = atan2((ROBOT_WIDTH / 2), MIN_SAFE_DISTANCE);
    if (first_pass) {
        //easier to use globals than dereference every time
        angle_min_ = laser_scan.angle_min;
        angle_increment_ = laser_scan.angle_increment;
        angle_cache_cos.resize(i);
        angle_cache_sin.resize(i);
        //precompute these
        while (i-- > 0) {
            // could use x,y,r as registers instead, but
            // this is more legible
            theta_i = angle_min_ + angle_increment_ * i;
            angle_cache_cos[i] = cos(theta_i);
            angle_cache_sin[i] = sin(theta_i);
        }
        i = laser_scan.ranges.size();
        first_pass = false;
    }
    laser_alarm_l = false;
    laser_alarm_r = false;

    while (i-- > 0) {
        r = laser_scan.ranges[i];
        x = r * angle_cache_cos[i];
        y = r * angle_cache_sin[i];
        if ((0 < x && x < MIN_SAFE_DISTANCE) && (-robot_radius < y && y < robot_radius)) {
            //if here, then in danger zone
            if (y >= 0) {
                laser_alarm_l = true;
            } else {
                laser_alarm_r = true;
            }
            ROS_WARN_THROTTLE(0.25, "Detected an object in %0.3f meters in front of me.", r);
            break; //no need to keep looking
        }
    }
    //be conservative
    lidar_alarm_msg.data = laser_alarm_l || laser_alarm_r;
    lidar_alarm_publisher_.publish(lidar_alarm_msg);
    //be direct
    lidar_alarm_msg.data = laser_alarm_l;
    lidar_alarm_publisher_l.publish(lidar_alarm_msg);
    lidar_alarm_msg.data = laser_alarm_r;
    lidar_alarm_publisher_r.publish(lidar_alarm_msg);
    if (laser_alarm_l || laser_alarm_r && !brake_enabled) {
        // stop the robot when alarm triggered
        robot.stop();
        brake_enabled = true;
    }
    if (brake_enabled && laser_alarm_l || laser_alarm_r) {
        // slowly move backward to release the lock
        robot.setMaxSpeed(0.1);
        robot.go(BACKWARD);
    }
    if (!(laser_alarm_l || laser_alarm_r) && brake_enabled) {
        // stop the backward movement
        robot.stop();
        brake_enabled = false;
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh;
    nh.param("MIN_SAFE_DISTANCE", MIN_SAFE_DISTANCE, MIN_SAFE_DISTANCE);
    nh.param("ROBOT_WIDTH", ROBOT_WIDTH, ROBOT_WIDTH);
    //create a Subscriber object and have it subscribe to the
    //lidar topic (made global so callback can use it)
    lidar_alarm_publisher_ = nh.advertise<std_msgs::Bool>("/lidar_alarm", 10);
    lidar_alarm_publisher_l = nh.advertise<std_msgs::Bool>("/lidar_alarm_l", 10);
    lidar_alarm_publisher_r = nh.advertise<std_msgs::Bool>("/lidar_alarm_r", 10);
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 10, laserCallback);

    ros::spin();
    return 0;
}
