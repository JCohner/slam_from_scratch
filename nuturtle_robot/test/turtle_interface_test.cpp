#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

ros::Publisher	vel_pub;
ros::Subscriber wheel_cmd_sub; 
bool x = true;
rigid2d::WheelVelocities wheel_vels;

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
	ASSERT_EQ(wheel_vels.left, 37);	
	ASSERT_EQ(wheel_vels.right, 44);	
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
}

int main(int argc, char * argv[])
{
	testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_node_name");
    setup();
    return RUN_ALL_TESTS();
}

