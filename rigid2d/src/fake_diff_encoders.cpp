#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

//define globals
double wheel_base;
double wheel_radius;
double freq;
ros::NodeHandle nh;
ros::Subscriber vel_sub;
ros::Publisher js_pub;
std::string left_wheel;
std::string right_wheel;
rigid2d::DiffDrive robot;

void vel_callback(geometry_msgs::Twist data){
	sensor_msgs::JointState msg;




	js_pub.publish(msg);
}
	
void setup(){
	//get public params
	nh.getParam("wheel/radius", wheel_radius);
	nh.getParam("wheel/base", wheel_base);
	nh.getParam("freq", freq);
	robot.set_wheel_props(wheel_radius, wheel_base);
	nh.getParam("~left_wheel_axel", left_wheel);
	nh.getParam("~right_wheel_axel", right_wheel);
	vel_sub = nh.subscribe("/turtle1/cmd_vel", 1, &vel_callback);
	js_pub = nh.advertise<sensor_msgs::JointState>("/joint_state",5);
}

void loop(){

	while(ros::ok()){
		ros::spinOnce();

	}
}


int main (int argc, char * argv[]){
	ros::init(argc, argv,"fake_diff_encoders");
	setup();
	loop();

	return 0;
}