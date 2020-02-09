#include <ros/ros.h>
#include "nuturtle_robot/Start.h"

/*Global Variables*/
ros::ServiceServer start;
ros::ServiceClient set_pose;


void setup()
{
	ros::NodeHandle nh;

}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "rotation");
	setup();

	return 0;
}