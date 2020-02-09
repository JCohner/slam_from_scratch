#include <ros/ros.h>
#include "nuturtle_robot/Start.h"
#include <turtlesim/TeleportAbsolute.h>

/*Global Variables*/
ros::ServiceServer start_srv;
ros::ServiceClient set_pose;
bool set_pose_available;

bool start_srv_callback(nuturtle_robot::Start::Request& req, nuturtle_robot::Start::Response& resp)
{
	//0 indicates CCW; 1 indicates CW traversal


	return true;
}

void setup()
{
	ros::NodeHandle nh;
	start_srv = nh.advertiseService("start", &start_srv_callback);
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

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "rotation");
	setup();

	return 0;
}