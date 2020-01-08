#include <ros/ros.h>
int main(int argc, char * argv[]){
	//initialize node
	ros::init(argc, argv, "hello_world");

	//create ros node handle
	ros::NodeHandle nh;

	ROS_INFO("DOINKS");
}