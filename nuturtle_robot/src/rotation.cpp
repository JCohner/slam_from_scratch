#include <ros/ros.h>
#include "nuturtle_robot/Start.h"
#include "rigid2d/rigid2d.hpp"
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>

/*Global Variables*/
ros::ServiceServer start_srv;
ros::ServiceClient set_pose;
ros::Publisher vel_pub;
bool set_pose_available;
int state;
int start = 0; //0 -> rotation, 1 paused
int num_rots = 0; 
double max_rot_vel = 2.84;
double frac_vel = 0.25; //TODO:make this a command line or param server parameter //fraction of max velocity
double rot_speed = frac_vel * max_rot_vel; //TODO: will have to move this to setup 
ros::Timer rot_timer, pause_timer;


bool start_srv_callback(nuturtle_robot::Start::Request& req, nuturtle_robot::Start::Response& resp)
{
	//0 indicates CCW; 1 indicates CW traversal
	start = 1;
	if (req.direction == 1)
	{
		rot_speed = -rot_speed;
	}

	return true;
}

void rot_timerCallback(const ros::TimerEvent& ev)
{
	if (!state && start)
	{
		state = !state; //set state to true
		geometry_msgs::Twist msg;
		msg.angular.z = rot_speed;
		vel_pub.publish(msg);
	}
}

void pause_timerCallback(const ros::TimerEvent& ev)
{
	if (state && start)
	{
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
	state = 0; //start robot in off state
	ros::NodeHandle nh;
	double rotation = 2 * rigid2d::PI;
	double rot_periodT = rotation/rot_speed;
	rot_timer = nh.createTimer(ros::Duration(rot_periodT), rot_timerCallback);
	rot_timer = nh.createTimer(ros::Duration(rot_periodT/20), pause_timerCallback);
	// nh.getParam("velocity/max_rot", max_rot_vel);
	start_srv = nh.advertiseService("start", &start_srv_callback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	if (ros::service::waitForService("set_pose", 1000)){
		set_pose_available = true;
		set_pose = nh.serviceClient<turtlesim::TeleportAbsolute>("set_pose");
		turtlesim::TeleportAbsolute pose;
		pose.request.theta = 0;
		pose.request.x = 0;
		pose.request.y = 0;
		set_pose.call(pose);

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