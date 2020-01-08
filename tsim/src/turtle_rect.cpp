#include <ros/ros.h>

int main(int argc, char * argv[]){
	//initialize node
	ros::init(argc, argv, "hello_world");

	//create ros node handle
	ros::NodeHandle nh;

	std::string hello_world;
	nh.getParam("/hello_world", hello_world);
	ROS_INFO(hello_world.c_str());
	// printf(hello_world);
}