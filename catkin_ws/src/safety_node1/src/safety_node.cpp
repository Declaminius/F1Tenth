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
        scan = n.subscribe("/scan", 1000, scan_callback);
        odom = n.subscribe("/odom", 1000, odom_callback);
        brake_pub = n.advertise<ackermann_msgs/AckermannDriveStamped>("/brake", 1000);
        brake_bool_pub = n.advertise<std_msgs/Bool>("/brake_bool", 1000);
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
    static void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        speed = odom_msg.twist.twist.linear.x;
    }

    static void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    	ackermann_msgs::AckermannDriveStamped brake_msg;
    	std_msgs::Bool brake_bool_msg;
    	
    	for (int i = 0; i <= 1080; i++) {
    	    auto current_angle = scan_msg.angle_min + i*scan_msg.angle_increment;
    	    auto projected_speed = speed*math.cos(current_angle);
    	    if (projected_speed > 0) {
		auto ttc = scan_msg.ranges[i]/projected_speed;    	    	
    	    }
    	    else { projected_speed = math.INFINITY }
    	    if (ttc < threshold) {
    	    	brake_msg.speed = 0;
    	    	brake_bool_msg = True;
    	    	brake_pub.publish(brake_msg);
    	    	brake_bool_pub.publish(brake_bool_msg);    	
	    }
        }
   }
};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}
