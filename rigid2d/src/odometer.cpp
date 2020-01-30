#include <ros/ros.h>
#include "diff_drive/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

//robot specs
double wheel_base;
double wheel_radius;
double freq;
rigid2d::DiffDrive robot;
std::string odom;
std::string body;
std::string left_wheel;
std::string right_wheel;

//ros node setup
ros::Subscriber js_sub;
ros::Publisher odom_pub;

//ros time management
ros::Time curr_time, prev_time;

//tf generation variables
double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void publishOdom(){
	static tf2_ros::TransformBroadcaster odom_broadcaster;
	curr_time = ros::Time::now();
	double dt = (curr_time - prev_time).toSec();
	double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
	// tf2::Quaternion odom_quat;
	// odom_quat.setRPY(0,0,th);

	//publish transform over tf using broadcaster
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = curr_time;
	odom_trans.header.frame_id = odom;
	odom_trans.child_frame_id = body;
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send transform
	odom_broadcaster.sendTransform(odom_trans);

	//set up odom message to be published over ROS this will make the transform between our baselink and the doom frame
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = curr_time;
	odom_msg.header.frame_id = odom;

	//set pos
	odom_msg.pose.pose.position.x = x;
	odom_msg.pose.pose.position.y = y;
	odom_msg.pose.pose.position.z = 0.0;
	odom_msg.pose.pose.orientation = odom_quat;

	//set vel
	odom_msg.child_frame_id = body;
	odom_msg.twist.twist.linear.x = vx;
	odom_msg.twist.twist.linear.y = vy;
	odom_msg.twist.twist.angular.z = vth;

	//publish odom message
	odom_pub.publish(odom_msg);

}

void js_callback(sensor_msgs::JointState data){
	robot.updateOdometry(data.position[0], data.position[1]);	
	publishOdom();
	return;
}

void setup(){
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	//get public params
	nh.getParam("wheel/radius", wheel_radius);
	nh.getParam("wheel/base", wheel_base);
	nh.getParam("freq", freq);
	robot.set_wheel_props(wheel_radius, wheel_base);
	//get private params
	nh_priv.getParam("frame_names/odom_frame_id", odom);
	nh_priv.getParam("frame_names/body_frame_id", body);
	nh_priv.getParam("frame_names/left_wheel_joint", left_wheel);
	nh_priv.getParam("frame_names/right_wheel_joint", right_wheel);

	js_sub = nh.subscribe("/joint_states", 1, &js_callback);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
}

void loop(){
	prev_time = ros::Time::now();
	curr_time = ros::Time::now();

	ros::Rate r(freq);
	while(ros::ok()){
		ros::spinOnce();

	}
}


int main(int argc, char * argv[]){
	ros::init(argc, argv,"odometer");
	setup();
	loop();


	return 0;
}
