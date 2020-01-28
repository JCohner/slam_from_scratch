#include <ros/ros.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <tsim/PoseError.h>

#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"
#include "waypoints/waypoints.hpp"

#include <stdio.h> //just for debugging right quick
//Define globals
ros::Subscriber pose_sub;
rigid2d::Waypoints waypoint;


// void init_telop(){
// 	//switch off pen and teleport to lower left corner
// 	turtlesim::SetPen pen_req;
// 	pen_req.request.off = 1;
// 	setPen_.call(pen_req);

// 	turtlesim::TeleportAbsolute tele_req;
// 	tele_req.request.x = rect_.x_; //TODO: grab waypoint 0 x 
// 	tele_req.request.y = rect_.y_; //TODO: grab waypoint 0 y 
// 	tele_req.request.theta = 0; //TODO: grab waypoint 0 theta

// 	teleportAbsolute_.call(tele_req);

// 	//switch on pen
// 	pen_req.request.off = 0;
// 	setPen_.call(pen_req);
// }

void pose_callback(turtlesim::Pose data){

}

void setup(){
	ros::NodeHandle nh;
	//populate waypoint data
	std::vector<std::vector<int>> points;
	std::vector<int> x(5,0);
	std::vector<int> y(5,0);
	nh.getParam("/waypoint_x", x);
	nh.getParam("/waypoint_y", y);
	unsigned int i;
	for (i = 0; i < points.size(); i++){
		points.push_back({x[i], y[i]});
	}
	// ROS_INFO("%d", points.size());
	// pose_sub = nh.subscribe("turtle1/pose", 1, &pose_callback);

	//move turtle to starting pos
	// init_telop();
}

int main(int argc, char * argv[]){
	ros::init(argc, argv,"turtle_way");
	setup();

	return 0;
}