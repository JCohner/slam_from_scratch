/// \file
/// \brief This implements the turtle interface node used for publishing wheel commands to and getting sensor data from the physical turtle, these are in response to commanded wheel velocities
///
/// PUBLISHES:
///     wheel_pub: <nuturtlebot::WheelCommands>: publishes to turtles wheels (makes em spin at requested vel) 
///		js_pub: <sensor_msgs::JointState> publishes joint states of wheels based on sensed encoder readings
/// SUBSCRIBES:
///     vel_sub: <geometry_msgs::Twist>: hears the commanded desired velocity
///		sensor_sub: <nuturtlebot::SensorData> information form the sensor 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

static ros::Subscriber sensor_sub;
static ros::Subscriber vel_sub;
static ros::Publisher wheel_pub;
static ros::Publisher js_pub;

/*Gloabl variables*/
static rigid2d::DiffDrive robot;

//grab these from param server eventually
static double wheel_base;
static double wheel_radius;
static double encoder_ticks_per_rev;
static double max_rot_vel;
static double max_trans_vel;
static double max_motor_speed; 
static std::string left_wheel;
static std::string right_wheel;
static double wheel_command_max;

void vel_sub_callback(geometry_msgs::Twist data)
{
	// ROS_INFO("%f", data.angular.z);
	//clamp inputs
	double angular_speed;
	double linear_speed;
	if (abs(data.angular.z) > max_rot_vel)
	{
		angular_speed = data.angular.z/fabs(data.angular.z) * max_rot_vel;
	} 
	else 
	{
		angular_speed = data.angular.z;
	}
	if (abs(data.linear.x) > max_trans_vel)
	{
		linear_speed = data.linear.x/fabs(data.linear.x) * max_trans_vel;
	} 
	else 
	{
		linear_speed = data.linear.x;
	}

	//convert commanded twist to wheel speeds
	rigid2d::WheelVelocities wheel_vels = robot.twistToWheels(rigid2d::Twist2D(angular_speed, linear_speed, 0));
	// ROS_INFO("commanded wheel vels: %f %f", wheel_vels.left, wheel_vels.right);
	//clamp wheel vels at no load motor speeds, map output between -265,265
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
	// ROS_INFO("eff_l %f, eff_r: %f", wheel_vels.left, wheel_vels.right);
	wheel_pub.publish(command);

	return;
}

void sens_sub_callback(nuturtlebot::SensorData data)
{
	sensor_msgs::JointState msg;
	double update[2] = {(double(data.left_encoder))/encoder_ticks_per_rev * 2 * rigid2d::PI,(double(data.right_encoder))/encoder_ticks_per_rev * 2 * rigid2d::PI};
	// double update[2] = {(double(data.left_encoder))/encoder_ticks_per_rev * 180,(double(data.right_encoder))/encoder_ticks_per_rev * 180};
	msg.position = {rigid2d::rad2deg(update[0]), rigid2d::rad2deg(update[1])};
	double encoders[2];
	robot.get_encoders(encoders);
	rigid2d::WheelVelocities vels(update[0] - encoders[0], update[1] - encoders[1]); 

	// robot.set_encoders(encoders[0] + update[0], encoders[1] + update[1]);
	robot.set_encoders(update[0], update[1]);

	msg.velocity = {vels.left, vels.right};
	msg.name = {left_wheel, right_wheel};
	msg.header.stamp = ros::Time::now();
	js_pub.publish(msg);
	return;
}


void setup()
{
	ros::NodeHandle nh;
	nh.getParam("/wheel/radius", wheel_radius);
	nh.getParam("/wheel/base", wheel_base);
	nh.getParam("/wheel/encoder_ticks_per_rev", encoder_ticks_per_rev);
	nh.getParam("/velocity/max_rot", max_rot_vel);
	nh.getParam("/velocity/max_trans", max_trans_vel);
	nh.getParam("/motor/no_load_speed", max_motor_speed);
	nh.getParam("/motor/max_power", wheel_command_max);
	nh.getParam("/odom/frame_names/left_wheel_joint", left_wheel);
	nh.getParam("/odom/frame_names/right_wheel_joint", right_wheel);

	robot.set_wheel_props(wheel_radius, wheel_base);
	robot.reset(rigid2d::Twist2D(0, 0, 0));
	robot.set_encoders(0, 0);
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