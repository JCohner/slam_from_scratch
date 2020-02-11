#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include <sensor_msgs/JointState.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

static ros::Publisher	vel_pub;
static ros::Subscriber wheel_cmd_sub; 
static ros::Publisher	sensor_pub;
static ros::Subscriber js_sub; 
static bool x;
static bool y;
static rigid2d::WheelVelocities wheel_vels;
static sensor_msgs::JointState js;

TEST(cmd_vel, no_rot)
{
	geometry_msgs::Twist msg;
	msg.angular.z = 0;
	msg.linear.x = 0.02;
	vel_pub.publish(msg);
	ros::Rate r(60);
	x = true;
	while(x)
	{
		r.sleep();
		ros::spinOnce();
	}

	ASSERT_EQ(wheel_vels.left, wheel_vels.right);
}

TEST(cmd_vel, pure_rot)
{
	geometry_msgs::Twist msg;
	msg.angular.z = 0.5;
	msg.linear.x = 0;
	vel_pub.publish(msg);
	ros::Rate r(60);
	x = true;
	while(x)
	{
		r.sleep();
		ros::spinOnce();
	}
	ASSERT_EQ(wheel_vels.left, -wheel_vels.right);	
}

TEST(cmd_vel, trans_and_rot)
{
	geometry_msgs::Twist msg;
	msg.angular.z = 0.5;
	msg.linear.x = 0.5;
	vel_pub.publish(msg);
	ros::Rate r(60);
	x = true;
	while(x)
	{
		r.sleep();
		ros::spinOnce();
	}
	ASSERT_EQ(wheel_vels.left, 226);	
	ASSERT_EQ(wheel_vels.right, 264);	
}

TEST(sens_sub, encoder_counts)
{
	nuturtlebot::SensorData msg;
	msg.left_encoder = 50; 
	msg.right_encoder = 60;
	sensor_pub.publish(msg);
	y = true; 
	ros::Rate r(60);
	while(y)
	{
		ros::spinOnce();
		r.sleep();
	}
	ASSERT_EQ(js.position[0], 50);
	ASSERT_EQ(js.position[1], 60);
	ASSERT_EQ(js.velocity[0], 50);
	ASSERT_EQ(js.velocity[1], 60);
}

void js_sub_callback(sensor_msgs::JointState data)
{
	y = false;
	js = data;
	return;
}

void wheel_cmd_sub_callback(nuturtlebot::WheelCommands data)
{
	x = false;
	wheel_vels.left = data.left_velocity;
	wheel_vels.right = data.right_velocity;

	return;
}

void setup()
{
	ros::NodeHandle nh;
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1, true);
	wheel_cmd_sub = nh.subscribe("wheel_cmd", 1, &wheel_cmd_sub_callback);
	sensor_pub = nh.advertise<nuturtlebot::SensorData>("sensor_data", 1,true);
	js_sub = nh.subscribe("joint_states", 1, &js_sub_callback);
}

int main(int argc, char * argv[])
{
	testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_node_name");
    setup();
    const auto x = RUN_ALL_TESTS();
    ros::shutdown();
    return x;
}

