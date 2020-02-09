#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

TEST(cmd_vel, no_rot)
{

}

ros::Publisher	vel_pub;
ros::Subscriber wheel_cmd_sub; 

void wheel_cmd_sub_callback(nuturtlebot::WheelCommands data)
{
	;
}

void setup()
{
	ros::NodeHandle nh;
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
	wheel_cmd_sub = nh.subscribe("wheel_cmd", 1, &wheel_cmd_sub_callback);
}


int main(int argc, char * argv[])
{
	testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_node_name");
    setup();
    return RUN_ALL_TESTS();
}

