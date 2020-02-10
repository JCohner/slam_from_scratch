#include <ros/ros.h>
#include "nuturtle_robot/Start.h"
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>

/*Global Variables*/
ros::ServiceServer start_srv;
ros::ServiceClient set_pose;
ros::Publisher vel_pub;
bool set_pose_available;
double freq = 0.25; //TODO:make this a command line or param server parameter

bool start_srv_callback(nuturtle_robot::Start::Request& req, nuturtle_robot::Start::Response& resp)
{
	//0 indicates CCW; 1 indicates CW traversal


	return true;
}

void setup()
{
	ros::NodeHandle nh;
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
	ros::Rate r(60);
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