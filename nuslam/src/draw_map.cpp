#include <ros/ros.h>
#include <nuslam/TurtleMap.h>
#include <visualization_msgs/Marker.h>

ros::Subscriber landmark_sub;

void landmark_sub_callback(nuslam::TurtleMap data)
{

}

void loop()
{
	
}


void setup()
{
	ros::NodeHandle nh;
	landmark_sub = nh.subscribe("landmarks", 1, &landmark_sub_callback);
}

int main(int argc, char * argv[]){
	ros::init(argc, argv, "landmarks");
	setup();
	loop();
}