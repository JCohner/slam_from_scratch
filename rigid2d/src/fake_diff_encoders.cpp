/// \file
/// \brief This implements the fake encoder node which listens for turtlesim commanded velocities and fakes encoder ticks which it publishes to join_states
///
/// PUBLISHES:
///     joint_states: <sensor_msgs::JointState>: faked encoder ticks translated from commanded velocities using twistToWheels() from rigid2d diffdrive
/// SUBSCRIBES:
///     cmd_vel: <geometry_msgs::Twist>: represents commanded velocity to diff drive turtle
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

//define globals
static double wheel_base;
static double wheel_radius;
static double freq;
static double encoder_ticks_per_rev;
static ros::Subscriber vel_sub;
static ros::Publisher js_pub;
static std::string left_wheel;
static std::string right_wheel;

//this robot translates commanded body twist to wheel twist
//tracks 'real change' in encoders
static rigid2d::DiffDrive robot;

void vel_callback(geometry_msgs::Twist data){
	// ROS_INFO("%f", data.angular.z);
	sensor_msgs::JointState msg;
	//we are representing this as 1/frequency of the twist 
	rigid2d::Twist2D Vb(data.angular.z, data.linear.x, 0);	
	// ROS_INFO("commanded body twist: omega: %f, vx: %f, vy: %f", Vb.omega, Vb.vel.x, Vb.vel.y);
	// rigid2d::WheelVelocities wheel_vels = robot.twistToWheels(Vb); //commanded velocities
	robot.feedforward(Vb);
	rigid2d::WheelVelocities wheel_vels = robot.wheelVelocities();
	double encoders[2];
	robot.get_encoders(encoders);
	robot.set_encoders(encoders[0] + wheel_vels.left, encoders[1] + wheel_vels.right); //simulate what the encoders read
	robot.get_encoders(encoders);
	msg.header.stamp = ros::Time::now();
	msg.name= {left_wheel, right_wheel};
	msg.position = {encoders[0], encoders[1]};
	msg.velocity = {wheel_vels.left, wheel_vels.right};
	js_pub.publish(msg); //publish as joint state message what the encoders read, this rotates wheels in rviz
}
	
void setup(){
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	//get public params
	nh.getParam("/wheel/radius", wheel_radius);
	nh.getParam("/wheel/base", wheel_base);
	nh.getParam("/wheel/encoder_ticks_per_rev", encoder_ticks_per_rev);
	nh.getParam("/freq", freq);
	robot.set_wheel_props(wheel_radius, wheel_base);
	robot.reset(rigid2d::Twist2D(0, 0, 0));
	robot.set_encoders(0,0);
	nh_priv.getParam("frame_names/left_wheel_joint", left_wheel);
	nh_priv.getParam("frame_names/right_wheel_joint", right_wheel);
	// vel_sub = nh.subscribe("/turtle1/cmd_vel", 1, &vel_callback);
	vel_sub = nh.subscribe("/cmd_vel", 1, &vel_callback); //switching to this for F.007
	js_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);
}

void loop(){
	ros::Rate r(freq);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
}


int main (int argc, char * argv[]){
	ros::init(argc, argv,"fake_diff_encoders");
	setup();
	loop();

	return 0;
}