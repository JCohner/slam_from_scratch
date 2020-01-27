#include <ros/ros.h>
#include "diff_drive/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double wheel_base;
double wheel_radius;
ros::NodeHandle nh;

void setup(){
	nh.getParam("/wheel/radius", wheel_radius);
	nh.getParam("/wheel/base", wheel_base);
}

void js_callback(sensor_msgs::JointState data){
	ROS_INFO("yuh wuh suh");
}

int main(int argc, char * argv[]){
	ros::init(argc, argv,"odometer");
	setup();
	ros::Subscriber js_sub = nh.subscribe("/joint_states", 1, &js_callback);
}