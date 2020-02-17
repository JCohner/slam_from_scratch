#include <ros/ros.h>
#include "waypoints/waypoints.hpp"
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"

//grab these from param server eventually
static double wheel_base;
static double wheel_radius;
static double encoder_ticks_per_rev;
static double max_rot_vel;
static double max_trans_vel;
static double max_motor_speed; 
static double freq;
static double frac_vel;
static double wheel_command_max;

rigid2d::Waypoints waypoints;
std::vector<double> curr_waypoint(3,0);
std::vector<double> x(5,0);
std::vector<double> y(5,0);
std::vector<double> th(5,0);
unsigned int i; 

void setup()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	//get robot properties
	nh.getParam("/wheel/radius", wheel_radius);
	nh.getParam("/wheel/base", wheel_base);
	nh.getParam("/wheel/encoder_ticks_per_rev", encoder_ticks_per_rev);
	nh.getParam("/velocity/max_rot", max_rot_vel);
	nh.getParam("/velocity/max_trans", max_trans_vel);
	nh.getParam("/motor/no_load_speed", max_motor_speed);
	nh.getParam("/motor/max_power", wheel_command_max);
	nh_priv.getParam("frac_vel",frac_vel);

	//get waypoint properties
	nh.getParam("/waypoint_x", x);
	nh.getParam("/waypoint_y", y);
	nh.getParam("/waypoint_th", th);
	nh.getParam("/freq", freq);

	waypoints = rigid2d::Waypoints(rigid2d::Transform2D(rigid2d::Vector2D(x[0],y[0]), th[0]), frac_vel* max_rot_vel, frac_vel* max_trans_vel, freq);
	for (unsigned int i = 0; i < x.size(); i++){
		waypoints.addWaypoint(std::vector<double> {x[i], y[i], th[i]}, i);
	}
	curr_waypoint = waypoints.get_curr_waypoint();
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv,"real_waypoint");
	setup();
	// loop();
	return 0;
}