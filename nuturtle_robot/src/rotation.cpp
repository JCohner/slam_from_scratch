#include <ros/ros.h>
#include "nuturtle_robot/Start.h"
#include "rigid2d/rigid2d.hpp"
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>

/*Global Variables*/
static ros::ServiceServer start_srv;
static ros::ServiceClient fake_set_pose;
static ros::ServiceClient set_pose;
static ros::Publisher vel_pub;
static bool set_pose_available;
static int state;
static int start = 0; //0 -> rotation, 1 paused
static int num_rots = 0; 
static double max_rot_vel = 2.84;
static double frac_vel = 0.25; //TODO:make this a command line or param server parameter //fraction of max velocity
static double rot_speed = frac_vel * max_rot_vel; //TODO: will have to move this to setup 
static ros::Timer rot_timer
static ros::Timer pause_timer;


bool start_srv_callback(nuturtle_robot::Start::Request& req, nuturtle_robot::Start::Response& resp)
{
	//0 indicates CCW; 1 indicates CW traversal
	start = 1;
	state = 0;
	if (req.direction == 1)
	{
		rot_speed = -rot_speed;
	}
	if (set_pose_available){
		turtlesim::TeleportAbsolute pose;
		pose.request.theta = 0;
		pose.request.x = 0;
		pose.request.y = 0;
		set_pose.call(pose);
		fake_set_pose.call(pose);
	}

	return true;
}

void rot_timerCallback(const ros::TimerEvent& ev)
{
	ROS_INFO("ROTATION triggering");
	if (!state && start)
	{
		// state = !state; //set state to true
		geometry_msgs::Twist msg;
		msg.angular.z = rot_speed;
		vel_pub.publish(msg);
	}
}

void pause_timerCallback(const ros::TimerEvent& ev)
{
	if (state && start)
	{
		ROS_INFO("PAUSE triggering");
		state = !state; //set state to false
		geometry_msgs::Twist msg;
		vel_pub.publish(msg);
		num_rots++;
	}

	if (start > 21)
	{
		start = 0;
	}
}

void setup()
{
	ros::NodeHandle nh;
	double rotation = 2 * rigid2d::PI;
	double rot_periodT = rotation/rot_speed;
	ROS_INFO("ROTATION: %f", rot_periodT);
	// pause_timer = nh.createTimer(ros::Duration(rot_periodT/20.0), pause_timerCallback);
	rot_timer = nh.createTimer(ros::Duration(rot_periodT), rot_timerCallback);
	// nh.getParam("velocity/max_rot", max_rot_vel);
	start_srv = nh.advertiseService("start", &start_srv_callback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	bool real_pose = ros::service::waitForService("set_pose");
	bool fake_pose = ros::service::waitForService("fake/set_pose");
	if (real_pose && fake_pose){
		set_pose_available = true;
		set_pose = nh.serviceClient<turtlesim::TeleportAbsolute>("set_pose");
		fake_set_pose = nh.serviceClient<turtlesim::TeleportAbsolute>("fake/set_pose");
		turtlesim::TeleportAbsolute pose;
		pose.request.theta = 0;
		pose.request.x = 0;
		pose.request.y = 0;
		set_pose.call(pose);
		fake_set_pose.call(pose);
		ROS_INFO("set pose!");
	} else {
		ROS_INFO("SetPose service is not available");
		set_pose_available = false;
	}
}

void loop()
{
	ros::Rate r(120); //TODO: get this from parameter server
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "rotation");
	setup();
	loop();
	return 0;
}