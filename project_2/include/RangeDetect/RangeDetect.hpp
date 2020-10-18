#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <math.h>

using namespace std;

class RangeDetect{

private:
  
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher range_pub_;
  
  // for /scan
  double scan_north_, scan_east_, scan_west_, scan_south_;
  vector<float> scan_data_;
  
  // for /odom
  double ang_z_;
  double pos_x_, pos_y_;
  
  // const
  double PI = 3.1415926;
  double max_dist_ = 4;
  
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg);
  void odomCallback(const nav_msgs::Odometry& odomMsg);
  void publish();
  
public:
  
  RangeDetect(ros::NodeHandle& nh);
  
};
