#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

//define globals
double wheel_base;
double wheel_radius;
double freq;
ros::Subscriber vel_sub;
ros::Publisher js_pub;
std::string left_wheel;
std::string right_wheel;
rigid2d::DiffDrive robot;

void vel_callback(geometry_msgs::Twist data){
	sensor_msgs::JointState msg;
	//we are representing this as 1/frequency of the twist 
	rigid2d::Twist2D Vb(data.angular.z/freq, data.linear.x/freq, 0);	
	rigid2d::WheelVelocities wheel_vels = robot.twistToWheels(Vb); //commanded velocities
	
	robot.updateOdometry(wheel_vels.left, wheel_vels.right); //simulate what the motors actually do
	double encoders[2];
	robot.get_encoders(encoders);
	robot.set_encoders(encoders[0] + wheel_vels.left, encoders[1] + wheel_vels.right); //simulate what the encoders read
	robot.get_encoders(encoders);
	msg.header.stamp = ros::Time::now();
	msg.name= {left_wheel, right_wheel};
	msg.position = {encoders[0], encoders[1]};
	ROS_INFO("%f %f", msg.position[0], msg.position[1]);
	js_pub.publish(msg);
}
	
void setup(){
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	//get public params
	nh.getParam("wheel/radius", wheel_radius);
	nh.getParam("wheel/base", wheel_base);
	nh.getParam("freq", freq);
	robot.set_wheel_props(wheel_radius, wheel_base);
	nh_priv.getParam("/odometer/frame_names/left_wheel_joint", left_wheel);
	nh_priv.getParam("/odometer/frame_names/right_wheel_joint", right_wheel);
	vel_sub = nh.subscribe("/turtle1/cmd_vel", 1, &vel_callback);
	js_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",5);
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