#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

/** Set alarm if anything is within 0.5m of the front of robot, or too
    close to the sides. */
double MIN_SAFE_DISTANCE = 1.0;
double ROBOT_RADIUS = 0.5;

/** these values to be set within the laser callback */
bool first_pass = true;
bool laser_alarm_l = false;
bool laser_alarm_r = false;
/** Precomputed for speed. */
double* angle_cache_sin;
double* angle_cache_cos;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_alarm_publisher_l;
ros::Publisher lidar_alarm_publisher_r;


void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
	std_msgs::Bool lidar_alarm_msg;
	int i = laser_scan.ranges.size();
	double angle_min_, angle_increment_, theta_i, x, y, r = 0.0;

	if (first_pass) {
		//easier to use globals than dereference every time
		angle_min_ = laser_scan.angle_min;
		angle_increment_ = laser_scan.angle_increment;
		//precompute these
		angle_cache_sin = new double[i];
		angle_cache_cos = new double[i];
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

	while (i-->0) {
		r = laser_scan.ranges[i];
		x = r * angle_cache_cos[i];
		y = r * angle_cache_sin[i];
		if ( (0 < x && x < MIN_SAFE_DISTANCE) &&
		     (-ROBOT_RADIUS < y && y < ROBOT_RADIUS) ) {
			//if here, then in danger zone
			if (y >= 0) {
				laser_alarm_l = true;
			} else {
				laser_alarm_r = true;
			}
			ROS_WARN("Detected an object in %0.2f meters in front of me.", r);
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
}

int main(int argc, char **argv) {
	if (argc < 3) {
		printf("Wrong number of arguments.  Using default values.\n"
			"Usage: lidar_alarm DIST RAD\n"
			"  DIST distance in front of robot to check\n"
			"  RAD  radius of robot\n"
		);
	} else {
		MIN_SAFE_DISTANCE = atof(argv[1]);
		ROBOT_RADIUS = atof(argv[2]);
	}
	ros::init(argc, argv, "lidar_alarm"); //name this node
	ros::NodeHandle nh;
    //create a Subscriber object and have it subscribe to the
    //lidar topic (made global so callback can use it)
    lidar_alarm_publisher_ = nh.advertise<std_msgs::Bool>("/lidar_alarm", 10);
    lidar_alarm_publisher_l = nh.advertise<std_msgs::Bool>("/lidar_alarm_l", 10);
    lidar_alarm_publisher_r = nh.advertise<std_msgs::Bool>("/lidar_alarm_r", 10);
	ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 10, laserCallback);

	ros::spin();
	return 0;
}
