/// \file
/// \brief This implements the turtle way node, a trajectory commanding node that has a turtlesim turtle follow a pentagram pattern 
///
/// PUBLISHES:
///     turtle1/cmd_vel: <geometry_msgs::Twist> commanded Vb of turtle
///		pose_error : <tsim::PoseError> represents error from desired position
/// SUBSCRIBES:
///     turtle/pose: <turtlesim::Pose> represents actual location of turtle
/// SERVICES:
///		CLIENTS: set_pen (turtlesim::SetPen) & teleport_absolute(turtlesim::TeleportAbsolute)
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
ros::Publisher pos_err_pub;

//vectors for holding waypoints
// std::vector<std::vector<int>> points(5, std::vector<int>(3, 0));
std::vector<double> curr_waypoint(3,0);
std::vector<double> x(5,0);
std::vector<double> y(5,0);
std::vector<double> th(5,0);
unsigned int i; 

void init_telop(){
	//switch off pen and teleport to lower left corner
	turtlesim::SetPen pen_req;
	pen_req.request.off = 1;
	setPen.call(pen_req);

	turtlesim::TeleportAbsolute tele_req;

	tele_req.request.x = curr_waypoint.at(0);  
	tele_req.request.y = curr_waypoint.at(1);
	tele_req.request.theta = 1.57079; 

	teleportAbsolute.call(tele_req);

	//switch on pen
	pen_req.request.off = 0;
	setPen.call(pen_req);
}

float x_actual;
float y_actual;
float theta_actual;
void pose_callback(turtlesim::Pose req){
	ROS_INFO("\nx: %f\ny: %f\nt: %f\n", req.x, req.y, req.theta);
	x_actual = req.x;
	y_actual = req.y;
	theta_actual = req.theta;
}

//wait for turtlesim_node services to become available
void wait_services(void){
	//wait for services to become available
	ros::service::waitForService("/turtle1/set_pen");
	ros::service::waitForService("/turtle1/teleport_absolute");
}

void setup(){
	ros::NodeHandle nh;
	//containers for params
	double rot_vel = 0; 
	double trans_vel = 0; 
	double freq = 0;

	nh.getParam("/waypoint_x", x);
	nh.getParam("/waypoint_y", y);
	nh.getParam("/waypoint_th", th);
	nh.getParam("/freq", freq);
	nh.getParam("/rot_vel", rot_vel);
	nh.getParam("/trans_vel", trans_vel);
	
	waypoints = rigid2d::Waypoints(rigid2d::Transform2D(rigid2d::Vector2D(x[0],y[0]), th[0]), rot_vel, trans_vel, freq);
	for (unsigned int i = 0; i < x.size(); i++){
		waypoints.addWaypoint(std::vector<double> {x[i], y[i], th[i]}, i);
	}
	curr_waypoint = waypoints.get_curr_waypoint();

	wait_services();
	//iniitalize publishers, services, and subscribers
	setPen = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
	teleportAbsolute = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",5);
	pose_sub = nh.subscribe("/turtle1/pose", 1, &pose_callback);
	pos_err_pub = nh.advertise<tsim::PoseError>("pose_error", 5);
	//move turtle to starting pos
	init_telop();
}

geometry_msgs::Twist VbToRos(rigid2d::Twist2D Vb){
	geometry_msgs::Twist twist;
	twist.linear.x = Vb.vel.x;
	twist.angular.z = Vb.omega;
	return twist;
}

double x_error;
double y_error;
double theta_error;
double x_ref;
double y_ref;
double th_ref;
double theta_refs[5] = {rigid2d::PI, rigid2d::PI/2, rigid2d::PI/4,  -rigid2d::PI/4, -rigid2d::PI/2};
void compute_error(){
	double t = waypoints.get_ellapsed(); 
	unsigned int state = waypoints.get_state();
	unsigned int index = waypoints.get_index(); 
	double angular_vel = waypoints.get_ang_vel();
	double trans_vel = waypoints.get_trans_vel();

	if (state % 2){
		//rot case
		x_ref = x[index];
		y_ref = y[index];
		switch(index){
			case 0:
				th_ref = angular_vel * t + rigid2d::PI;
				break;
			case 1:
				th_ref = angular_vel * t +  rigid2d::PI/2;
				break;
			case 2:
				th_ref = angular_vel * t +  rigid2d::PI/4;
				break;
			case 3:
				th_ref = angular_vel * t -rigid2d::PI/4;
				break;
			case 4:
				th_ref = angular_vel * t - rigid2d::PI/2;
				break;
			default:
				break;
		}
	} else {
		th_ref = theta_refs[index]; 
		switch(index){
			case 0:
				x_ref = -trans_vel * t + x[4];
				y_ref = y[index];
				break;
			case 1:
				x_ref = x[index];
				y_ref = trans_vel * t + y[index-1];
				break;
			case 2:
				x_ref = trans_vel * t + x[index-1];
				y_ref = trans_vel * t + y[index-1];
				break;
			case 3:
				x_ref = trans_vel * t + x[index-1];
				y_ref = -trans_vel * t + y[index-1];
				break;
			case 4:
				x_ref = x[index-1];
				y_ref = -trans_vel * t + y[index-1];
				break;
			default:
				break;
		}
	}
	x_error = abs(x_ref - x_actual);
	y_error = abs(y_ref - y_actual);
	theta_error = abs(th_ref - theta_actual);
	tsim::PoseError msg;
	msg.x_error = x_error;
	msg.y_error = y_error;
	msg.theta_error = theta_error;
	pos_err_pub.publish(msg);
}

void loop(){
	ros::Rate r(waypoints.get_freq());
	geometry_msgs::Twist cmd_twist;
	while(ros::ok()){
		cmd_twist = VbToRos(waypoints.nextWaypoint());
		vel_pub.publish(cmd_twist);
		compute_error();
		ros::spinOnce();
		r.sleep();
	}
}


int main(int argc, char * argv[]){
	ros::init(argc, argv,"turtle_way");
	setup();
	waypoints.start();
	loop();
	return 0;
}