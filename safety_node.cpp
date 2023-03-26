#include <ackermann_msgs/AckermannDriveStamped.h>
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <safety_node1/safety_nodeConfig.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>    
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

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
  ros::Publisher car_speed_pub;
  ros::Publisher ttc_pub;
  ros::Publisher emergency_brake_indicator_pub;
  ros::Publisher mdtao_pub;
  double threshold;
  int embi=1;
  double mdtao = 100000;   //minimum distance to any obstacle, set to a high number
public:
  Safety(ros::NodeHandle handle) {
    n = handle;
    scan = n.subscribe("/scan", 1000, &Safety::scan_callback, this);
    odom = n.subscribe("/odom", 1000, &Safety::odom_callback, this);
    brake_pub =
        n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);
    brake_bool_pub = n.advertise<std_msgs::Bool>("/brake_bool", 1000);
    car_speed_pub = n.advertise<std_msgs::Float64>("/car_speed_topic", 1000); //publisher for velocity
    ttc_pub = n.advertise<std_msgs::Float64>("/ttc_topic", 1000);            //publisher for ttc
    emergency_brake_indicator_pub = n.advertise<std_msgs::Int8>("/embi_topic", 1000);
    mdtao_pub = n.advertise<std_msgs::Float64>("/mdtao_topic", 1000);
    speed = 0.0;
    threshold = 0.4;

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
    
    min_ttc = 100;
    for (int i = 0; i <= 1080; i++) {
      auto current_angle = scan_msg.angle_min + i * scan_msg.angle_increment;
      auto projected_speed = speed * cos(current_angle);
      
      std_msgs::Float64 msg;             //can not publish directly that's why we have this code here
      msg.data = projected_speed;
      car_speed_pub.publish(msg);
      
      if (projected_speed > 0) {
        ttc = scan_msg.ranges[i] / projected_speed;
        
        std_msgs::Float64 msg_ttc;      //same as before
        msg_ttc.data = embi;
        ttc_pub.publish(msg_ttc);
        
      if(scan_msg.ranges[i]<mdtao){
     	 mdtao = scan_msg.ranges[i];
      }
        
        if (ttc < min_ttc) {
          min_ttc = ttc;
        }
      }
    
    if(scan_msg.ranges[i]<mdtao){
     	 mdtao = scan_msg.ranges[i];
      }
    
    }
    
        std_msgs::Float64 msg_mdtao;      //same as before
        msg_mdtao.data = mdtao;
        mdtao_pub.publish(msg_mdtao);
    
    
    
    
    if (min_ttc < threshold && speed > 0) {
      brake_msg.drive.speed = 0;
      brake_bool_msg.data = true;
      brake_pub.publish(brake_msg);
      brake_bool_pub.publish(brake_bool_msg);
      
        std_msgs::Int8 msg_embi;      //same as before
        msg_embi.data = embi;
        emergency_brake_indicator_pub.publish(msg_embi);
      
      
      
      
    } else {
      brake_bool_msg.data = false;
      brake_bool_pub.publish(brake_bool_msg);
    }
  }

  void dyn_callback(const safety_node1::safety_nodeConfig &config,
                    uint32_t level) {
    threshold = config.threshold;
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "safety_node");
  ros::NodeHandle n = ros::NodeHandle();
  Safety sn(n);

  dynamic_reconfigure::Server<safety_node1::safety_nodeConfig> server;
  dynamic_reconfigure::Server<safety_node1::safety_nodeConfig>::CallbackType f;

  f = boost::bind(&Safety::dyn_callback, &sn, _1, _2);
  server.setCallback(f);

  ros::spin();
  return 0;
}
