#include "BotControl.hpp"
#include <ros/ros.h>

using namespace botcontrol;

BotControl::BotControl(ros::NodeHandle& nh) : nodehandle_(nh){

	//load the param
	if(!loadParam()){
		ROS_ERROR("Error in loading the parameters.");
		// ros::requestShutdown();
	}

	// declare all the subscriber and publisher
	scan_sub_ = nodehandle_.subscribe("/scan", 1, &BotControl::scanCallBack, this);
	odom_sub_ = nodehandle_.subscribe("/husky_velocity_controller/odom", 1, &BotControl::odomCallBack, this);

	vel_pub_ = nodehandle_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 200);
	error_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_forward", 1); 
	error_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/error_angle", 1);
	control_signal_forward_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_forward", 1);
	control_signal_angle_pub_ = nodehandle_.advertise<std_msgs::Float32>("/control_signal_angle", 1);

	//initialize variables
	error_forward_ = 0;
	error_angle_ = 0;
	error_forward_prev_ = 0;
	error_angle_prev_ = 0;
	I_forward_ = 0;
	I_angle_ = 0;
	D_forward_ = 0;
	D_angle_ = 0;

	ROS_INFO("Node Initialized");
}

BotControl::~BotControl(){}

void BotControl::odomCallBack(const nav_msgs::OdometryConstPtr& odomMsg){
	pos_x_ = odomMsg->pose.pose.position.x;
	pos_y_ = odomMsg->pose.pose.position.y;
	q_z_ = odomMsg->pose.pose.orientation.z;
	ang_z_ = q_z_ * 2.19;

	// ROS_INFO("/odom receive : x: %f, y: %f, z: %f", pos_x_, pos_y_, ang_z_);

}

void BotControl::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scanMsg){
	scan_data_ = scanMsg->ranges;
	int arr_size = scan_data_.size();
	float smallest_dist = 100;

	for(int i = 0; i<arr_size; i++){
		if(scan_data_[i] < smallest_dist) {
			smallest_dist = scan_data_[i];
			scan_ang_ = scanMsg->angle_min + scanMsg->angle_increment*i;
		}
	}
	scan_range_ = smallest_dist;

	// ROS_INFO("/scan receive : %f", scan_range_);

	pidAlgorithm();
}

void BotControl::pidAlgorithm(){

	std_msgs::Float32 linear_error;
	std_msgs::Float32 angle_error;
	std_msgs::Float32 linear_velocity;
	std_msgs::Float32 angle_velocity;

	// update the pid status
	error_forward_prev_ = error_forward_;
	error_angle_prev_ = error_angle_;
	error_forward_ = -scan_range_ + target_distance;
	// error_angle_ = ang_z_ - target_angle; //original
	error_angle_ = -scan_ang_ + target_angle;


	// regularize the error_angle_ within [-PI, PI]
	if(error_angle_ < -PI) error_angle_ += 2*PI;
	if(error_angle_ > PI) error_angle_ -= 2*PI;

	// integral term
	I_forward_ += dt * error_forward_;
	I_angle_ += dt * error_angle_;

	// derivative term
	D_forward_ = (-error_forward_prev_ + error_forward_) / dt;
	D_angle_ = (-error_angle_prev_ + error_angle_) / dt;

	if(error_forward_ > 0.01){ // the threshold is set to be 0.2
		trans_forward_ = Kp_f * error_forward_ + Ki_f * I_forward_ + Kd_f * D_forward_;
	} else if(error_forward_ < -0.01){
		trans_forward_ = -Kp_f * error_forward_ + -Ki_f * I_forward_ + -Kd_f * D_forward_;
	} else{
		trans_forward_ = 0;
	}
	// set threshold
	if(trans_forward_ > 2) trans_forward_ = 2;
	if(trans_forward_ < -2) trans_forward_ = -2;

	if(error_angle_ > 0.02){
		trans_angle_ = Kp_a * error_angle_ + Ki_a * I_angle_ + Kd_a * D_angle_;
	} else if(error_angle_ < -0.02){
		trans_angle_ = -Kp_a * error_angle_ + -Ki_a * I_angle_ + -Kd_a * D_angle_;
	} else{
		trans_angle_ = 0;
	}

	ROS_INFO("Forward Velocity: %f; Angle Velocity: %f; Orientation_error: %f, Distance: %f", 
		trans_forward_, trans_angle_, error_angle_, scan_range_);

	//publish all
	vel_cmd_.linear.x = trans_forward_;
	vel_cmd_.angular.z = trans_angle_; //eular angle
	vel_pub_.publish(vel_cmd_);

	linear_error.data = error_forward_;
	error_forward_pub_.publish(linear_error);

	linear_velocity.data = trans_forward_;
	control_signal_angle_pub_.publish(linear_velocity);

	angle_error.data = error_angle_;
	error_angle_pub_.publish(angle_error);

	angle_velocity.data = trans_angle_;
	control_signal_angle_pub_.publish(angle_velocity);

}

void BotControl::spin(){
	ros::Rate loop_rate(1/dt);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

}

bool BotControl::loadParam(){


	if(!nodehandle_.getParam("/Kp_f", Kp_f)){
		ROS_ERROR("Kp_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Ki_f", Ki_f)){
		ROS_ERROR("Ki_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kd_f", Kd_f)){
		ROS_ERROR("Kd_f Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kp_a", Kp_a)){
		ROS_ERROR("Kp_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Ki_a", Ki_a)){
		ROS_ERROR("Ki_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/Kd_a", Kd_a)){
		ROS_ERROR("Kd_a Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/target_distance", target_distance)){
		ROS_ERROR("target_distance Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/target_angle", target_angle)){
		ROS_ERROR("target_angle Load Error");
		return false;
	}
	if(!nodehandle_.getParam("/dt", dt)){
		ROS_ERROR("dt Load Error");
		return false;
	}

	return true;

}