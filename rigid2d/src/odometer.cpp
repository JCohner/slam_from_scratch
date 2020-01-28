#include <ros/ros.h>
#include "diff_drive/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
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
ros::NodeHandle nh;
ros::Subscriber js_sub;
ros::Publisher odom_pub;
// tf2_ros::TransformBroadcaster odom_broadcaster;

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
	robot.updateOdometry(data.position[0], data.position[1]);	
	nav_msgs::Odometry msg;
	msg.header.frame_id = odom;
	msg.child_frame_id = body;
	odom_pub.publish(msg);
	
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = odom;
	transformStamped.child_frame_id = body;
	transformStamped.transform.translation.x = data.position[0]; //see if my index guess is right
	transformStamped.transform.translation.y = data.position[1];
	transformStamped.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0,0,data.position[2]); //really not sure if this correlates to theta or z, supposed to be theta
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
	br.sendTransform(transformStamped);

	return;
}

void setup(){
	//get public params
	nh.getParam("wheel/radius", wheel_radius);
	nh.getParam("wheel/base", wheel_base);
	nh.getParam("freq", freq);
	robot.set_wheel_props(wheel_radius, wheel_base);
	//get private params
	nh.getParam("~odom", odom);
	nh.getParam("~base_link", body);
	nh.getParam("~left_wheel_axel", left_wheel);
	nh.getParam("~right_wheel_axel", right_wheel);

	js_sub = nh.subscribe("/joint_states", 1, &js_callback);
	odom_pub = nh.advertise<nav_msgs::Odometry>("/tf", 1);

}


void loop(){
	prev_time = ros::Time::now();
	curr_time = ros::Time::now();

	ros::Rate r(freq);
	while(ros::ok()){
		ros::spinOnce();
		// curr_time = ros::Time::now();
		// double dt = (curr_time - prev_time).toSec();

	}
}


int main(int argc, char * argv[]){
	ros::init(argc, argv,"odometer");
	setup();
	loop();


	return 0;
}
