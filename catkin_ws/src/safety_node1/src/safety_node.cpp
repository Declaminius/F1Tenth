#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>

// TODO: include ROS msg type headers and libraries

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    ros::Subscriber scan;
    ros::Subscriber odom;
    ros::Publisher brake_pub;
    ros::Publisher brake_bool_pub;

public:
    Safety() {
        n = ros::NodeHandle();
        scan = n.subscribe("/scan", 1000, &Safety::scan_callback, this);
        odom = n.subscribe("/odom", 1000, &Safety::odom_callback, this);
        brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);
        brake_bool_pub = n.advertise<std_msgs::Bool>("/brake_bool", 1000);
        speed = 0.0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */
        
    }
    void odom_callback(const nav_msgs::Odometry &odom_msg) {
        speed = odom_msg.twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan &scan_msg) {
    	ackermann_msgs::AckermannDriveStamped brake_msg;
    	std_msgs::Bool brake_bool_msg;
    	double ttc;
    	double min_ttc;
    	double threshold = 0.4;
    	
    	min_ttc = 100;
    	for (int i = 0; i <= 1080; i++) {
    	    auto current_angle = scan_msg.angle_min + i*scan_msg.angle_increment;
    	    auto projected_speed = speed*cos(current_angle);
    	    if (projected_speed > 0) {
    	    	ttc = scan_msg.ranges[i]/projected_speed;
    	    	if (ttc < min_ttc) {
    	            min_ttc = ttc;
    	    	}    	    	
    	    }
        }
        if (min_ttc < threshold && speed > 0) {
    	    brake_msg.drive.speed = 0;
    	    brake_bool_msg.data = true;
    	    brake_pub.publish(brake_msg);
    	    brake_bool_pub.publish(brake_bool_msg);    	
	}
	else {
	   brake_bool_msg.data = false;
	   brake_bool_pub.publish(brake_bool_msg);
	}
   }
};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    sn = Safety();
    ros::spin();
    return 0;
}
