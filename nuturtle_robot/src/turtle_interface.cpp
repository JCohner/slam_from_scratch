#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
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
std::string left_wheel = "left_wheel_axel";
std::string right_wheel = "right_wheel_axel";

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
	double wheel_command_max = 44;
	//clamp wheel vels at no load motor speeds, map output between -44,44
	//clamp
	wheel_vels.left = wheel_vels.left / max_motor_speed * wheel_command_max;
	wheel_vels.right = wheel_vels.right / max_motor_speed * wheel_command_max;
	if (abs(wheel_vels.left) > wheel_command_max)
	{
		wheel_vels.left = wheel_vels.left / abs(wheel_vels.left) * wheel_command_max;
	}
	if (abs(wheel_vels.right) > wheel_command_max)
	{
		wheel_vels.right = wheel_vels.right / abs(wheel_vels.right) * wheel_command_max;
	}

	nuturtlebot::WheelCommands command;
	command.left_velocity = wheel_vels.left;
	command.right_velocity = wheel_vels.right;

	wheel_pub.publish(command);

	return;
}

void sens_sub_callback(nuturtlebot::SensorData data)
{
	sensor_msgs::JointState msg;
	msg.position = {data.left_encoder, data.right_encoder};
	msg.velocity = 
	msg.name = {left_wheel, right_wheel}
	msh.header.stamp = ros::Time::now();
	js_pub.publish(msg);
	return;
}


void setup()
{
	ros::NodeHandle nh;
	//TODO: get params from param server

	robot.set_wheel_props(wheel_radius, wheel_base);
	wheel_pub = nh.advertise<nuturtlebot::WheelCommands>("wheel_cmd",1);
	js_pub = nh.advertise<sensor_msgs::JointState>("/joint_states",1);
	vel_sub = nh.subscribe("cmd_vel", 1,  &vel_sub_callback); //check what the actual topic name is
	sensor_sub = nh.subscribe("sensor_data", 1, &sens_sub_callback);
}


void loop()
{
	ros::Rate r(60);
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv,"turtle_interface");
	setup();
	loop();
	return 0;

}