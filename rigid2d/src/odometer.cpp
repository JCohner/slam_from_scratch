#include <ros/ros.h>
#include "diff_drive/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//robot specs
double wheel_base;
double wheel_radius;
double freq;

//ros node setup
ros::NodeHandle nh;
ros::Subscriber js_sub;
ros::Publisher odom_pub;
tf::TransformBroadcaster odom_broadcaster;

//ros time management
ros::Time curr_time, prev_time;

//tf generation variables
double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void js_callback(sensor_msgs::JointState data){
	ROS_INFO("yuh wuh suh");
}

void setup(){
	nh.getParam("/wheel/radius", wheel_radius);
	nh.getParam("/wheel/base", wheel_base);
	nh.getParam("/freq", freq);
	js_sub = nh.subscribe("/joint_states", 1, &js_callback);
	odom_pub = nh.advertise<nav_msgs::Odometry>("/tf", 1);
}


void loop(){
	prev_time = ros::Time::now();
	curr_time = ros::Time::now();

	ros::Rate r(freq);
	while(ros::ok()){
		ros::spinOnce();
		curr_time = ros::Time::now();
	}
}


int main(int argc, char * argv[]){
	ros::init(argc, argv,"odometer");
	setup();
	loop();


	return 0;
}
