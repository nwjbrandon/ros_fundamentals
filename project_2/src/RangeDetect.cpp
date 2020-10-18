#include "RangeDetect/RangeDetect.hpp"

RangeDetect::RangeDetect(ros::NodeHandle& nh)
{
  nh_ = nh;
  scan_sub_ = nh_.subscribe("/scan", 1, &RangeDetect::scanCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &RangeDetect::odomCallback, this);
  range_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/range_pub", 1);

  ROS_INFO("range_detect_node initialized successfully");
}

/*
 * scanCallback Function
 * @param: scanMsg 
 * @remark: this function is intended to be applied on husky_simulator, the laser scan range (deg) is -145 to 145
 */
void RangeDetect::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scanMsg)
{
  scan_data_ = scanMsg->ranges;
  double scan_increment = scanMsg->angle_increment;
  
  int scan_size = scan_data_.size();
  int east_index = 0, north_index = scan_size/4, west_index = scan_size/2, south_index = scan_size*3/4;
  
  int increment = (int) (ang_z_ / scan_increment);
  
  east_index -= increment;
  north_index -= increment;
  west_index -= increment;
  south_index -= increment;
  
  while(east_index < 0) east_index += scan_size;
  while(north_index < 0) north_index += scan_size;
  while(west_index < 0) west_index += scan_size;
  while(south_index < 0) south_index += scan_size;
  
  while(east_index > scan_size) east_index -= scan_size;
  while(north_index > scan_size) north_index -= scan_size;
  while(west_index > scan_size) west_index -= scan_size;
  while(south_index > scan_size) south_index -= scan_size;
  
  scan_east_ = scan_data_[east_index];
  scan_north_ = scan_data_[north_index];
  scan_west_ = scan_data_[west_index];
  scan_south_ = scan_data_[south_index];
  
  if(isinf(scan_east_)) scan_east_ = max_dist_;
  if(isinf(scan_north_)) scan_north_ = max_dist_;
  if(isinf(scan_west_)) scan_west_ = max_dist_;
  if(isinf(scan_south_)) scan_south_ = max_dist_;
  
  //ROS_INFO("increment: %d", increment);
  //ROS_INFO("index, E:%d, N:%d, W:%d, S:%d", east_index, north_index, west_index, south_index);
  ROS_INFO("E:%f, N:%f, W:%f, S:%f", scan_east_, scan_north_, scan_west_, scan_south_);
  
  publish();
}


void RangeDetect::odomCallback(const nav_msgs::Odometry& odomMsg)
{
  double qx = odomMsg.pose.pose.orientation.x;
  double qy = odomMsg.pose.pose.orientation.y;
  double qz = odomMsg.pose.pose.orientation.z;
  double qw = odomMsg.pose.pose.orientation.w;
  
  ang_z_ = atan2(2*(qw*qz+qx*qy), 1-2*(qz*qz+qy*qy));
  
  pos_x_ = odomMsg.pose.pose.position.x;
  pos_y_ = odomMsg.pose.pose.position.y;
  
  //ROS_INFO("ang_z_:%f, pos_x_:%f, pos_y_:%f", ang_z_/PI*180, pos_x_, pos_y_);
  
}

void RangeDetect::publish()
{
  std_msgs::Float32MultiArray msg_pub;
  msg_pub.data.clear();
  msg_pub.data.push_back(scan_north_);
  msg_pub.data.push_back(scan_east_);
  msg_pub.data.push_back(scan_south_);
  msg_pub.data.push_back(scan_west_);
  range_pub_.publish<std_msgs::Float32MultiArray>(msg_pub);
  
}




