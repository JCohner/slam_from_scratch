#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

ros::Subscriber sensor_sub;
ros::Subscriber vel_sub;
ros::Publisher wheel_pub;
ros::Publisher js_pub;

/*Gloabl variables*/
rigid2d::DiffDrive robot;

//grab these from param server eventually
double wheel_base = 0.16 ;
double wheel_radius = 0.033;
double max_rot_vel = 2.84; //rad/s
double max_trans_vel = 0.22; //m/s
double max_motor_speed = 6.35492; //rad/s

void vel_sub_callback(geometry_msgs::Twist data)
{
	//clamp inputs
	double angular_speed;
	double linear_speed;
	if (abs(data.angular.z) > max_rot_vel)
	{
		angular_speed = data.angular.z/abs(data.angular.z) * max_rot_vel;
	} 
	else 
	{
		angular_speed = data.angular.z;
	}
	if (abs(data.linear.x) > max_trans_vel)
	{
		linear_speed = data.linear.x/abs(data.linear.x) * max_trans_vel;
	} 
	else 
	{
		linear_speed = data.linear.x;
	}

	//convert commanded twist to wheel speeds
	rigid2d::WheelVelocities wheel_vels = robot.twistToWheels(rigid2d::Twist2D(angular_speed, linear_speed, 0));

	//clamp wheel vels at no load motor speeds, map output between -44,44
	if (abs(wheel_vels.left) > max_motor_speed)
	{
		wheel_vels.left = wheel_vels.left / abs(wheel_vels.left) * max_motor_speed;
	}
	if (abs(wheel_vels.right) > max_motor_speed)
	{
		wheel_vels.right = wheel_vels.right / abs(wheel_vels.right) * max_motor_speed;
	}

	return;
}

void setup()
{
	robot.set_wheel_props(wheel_radius, wheel_base);
	ros::NodeHandle nh;
	wheel_pub = nh.advertise<nuturtlebot::WheelCommands>("wheel_cmd",1);
	js_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
	vel_sub = nh.subscribe("cmd_vel", 1,  &vel_sub_callback); //check what the actual topic name is
}


int main(int argc, char * argv[])
{
	ros::init(argc, argv,"turtle_interface");
	setup();

	return 0;

}