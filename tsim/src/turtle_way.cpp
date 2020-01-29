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
rigid2d::Waypoints waypoints;
ros::ServiceClient setPen;
ros::ServiceClient teleportAbsolute;
ros::Publisher vel_pub;

//vectors for holding waypoints
// std::vector<std::vector<int>> points(5, std::vector<int>(3, 0));
std::vector<double> curr_waypoint(3,0);


//define rot and trans vels
const double rot_vel = 0.5; //rad/s
const double trans_vel = 0.5; //m/s

//time trackers
double freq = 60;
unsigned int i; 

void init_telop(){
	//switch off pen and teleport to lower left corner
	turtlesim::SetPen pen_req;
	pen_req.request.off = 1;
	setPen.call(pen_req);

	turtlesim::TeleportAbsolute tele_req;

	tele_req.request.x = curr_waypoint.at(0);  
	tele_req.request.y = curr_waypoint.at(1);
	tele_req.request.theta = curr_waypoint.at(2); 

	teleportAbsolute.call(tele_req);

	//switch on pen
	pen_req.request.off = 0;
	setPen.call(pen_req);
}

void pose_callback(turtlesim::Pose data){
	;
}

//wait for turtlesim_node services to become available
void wait_services(void){
	//wait for services to become available
	ros::service::waitForService("/turtle1/set_pen");
	ros::service::waitForService("/turtle1/teleport_absolute");
}

void setup(){
	ros::NodeHandle nh;
	//get parameters
	std::vector<double> x(5,0);
	std::vector<double> y(5,0);
	std::vector<double> th(5,0);
	nh.getParam("/waypoint_x", x);
	nh.getParam("/waypoint_y", y);
	nh.getParam("/waypoint_th", th);
	for (unsigned int i = 0; i < x.size(); i++){
		waypoints.addWaypoint(std::vector<double> {x[i], y[i], th[i]}, i);
	}
	curr_waypoint = waypoints.get_curr_waypoint();
	wait_services();
	//iniitalize publishers, services, and subscribers
	setPen = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
	teleportAbsolute = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",5);

	//move turtle to starting pos
	init_telop();
}

void loop(){

	ros::Rate r(freq);
	while(ros::ok()){
		
		// compute_error();
		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char * argv[]){
	ros::init(argc, argv,"turtle_way");
	setup();
	loop();
	return 0;
}