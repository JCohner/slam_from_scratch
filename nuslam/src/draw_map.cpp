#include <ros/ros.h>
#include <nuslam/TurtleMap.h>
#include <visualization_msgs/Marker.h>

ros::Subscriber landmark_sub;
ros::Publisher marker_pub;

visualization_msgs::Marker mark;
int i = 0;
int num_expected = 8;
uint32_t shape = visualization_msgs::Marker::CYLINDER;

void landmark_sub_callback(nuslam::TurtleMap data)
{
	mark.header.frame_id = "base_scan";
	mark.header.stamp = ros::Time::now();
	mark.ns = "mork";
	mark.id = i;
	i = (i + 1) % num_expected;
	mark.type = shape;
	mark.action = visualization_msgs::Marker::ADD;
	mark.pose.position.x = data.centerX;
	mark.pose.position.y = data.centerY;
	mark.pose.position.z = 0;
	mark.pose.orientation.x = 0.0;
	mark.pose.orientation.y = 0.0;
	mark.pose.orientation.z = 0.0;
	mark.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	//TODO: scale these properly
	mark.scale.x = 0.2;
	mark.scale.y = 0.2;
	mark.scale.z = 0.2;


	// Set the color -- be sure to set alpha to something non-zero!
	mark.color.r = 0.0f;
	mark.color.g = 1.0f;
	mark.color.b = 0.0f;
	mark.color.a = 1.0;
	mark.lifetime = ros::Duration();

	marker_pub.publish(mark);
}

// void loop()
// {
	
// }


void setup()
{
	ros::NodeHandle nh;
	landmark_sub = nh.subscribe("landmarks", 1, &landmark_sub_callback);
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
}

int main(int argc, char * argv[]){
	ros::init(argc, argv, "draw_map");
	setup();
	// loop();
	ros::spin();
}