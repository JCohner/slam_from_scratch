#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nuslam/TurtleMap.h>

ros::Publisher landmark_pub;

void loop()
{

	ros::Rate r(60);
	while(ros::ok())
	{
		r,sleep();
		ros.spinOnce();
	}
}


void setup()
{
	ros::NodeHandle nh;
	landmark_pub = nh.advertise<nuslam::TurtleMap>("landmarks", 1);
}

int main(int argc, char * argv[]){
	ros::init(argc, argv, "landmarks");
	setup();
	loop();
}