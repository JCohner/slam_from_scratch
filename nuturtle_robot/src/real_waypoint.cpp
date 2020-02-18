#include <ros/ros.h>
#include "waypoints/waypoints.hpp"
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"
#include <visualization_msgs/Marker.h>

//grab these from param server eventually
/*robot & general properties*/
static double wheel_base;
static double wheel_radius;
static double encoder_ticks_per_rev;
static double max_rot_vel;
static double max_trans_vel;
static double max_motor_speed; 
static double freq;
static double frac_vel;
static double wheel_command_max;

/*waypoint structures*/
rigid2d::Waypoints waypoints;
std::vector<double> curr_waypoint(3,0);
std::vector<double> x(5,0);
std::vector<double> y(5,0);
std::vector<double> th(5,0);
unsigned int i; 

/*marker pub*/
ros::Publisher marker_pub;


void pub_marker()
{
	uint32_t shape = visualization_msgs::Marker::CYLINDER;
	std::vector<visualization_msgs::Marker> marks(5);
	std::vector<std::string> frame_names = {"/marker0", "/marker1", "/marker2", "/marker3", "/marker4"}; 
	int i = 0;
	ros::Rate r(60);
	for (auto mark : marks)
	{
		curr_waypoint = waypoints.get_waypoint_at(i);
		mark.header.frame_id = frame_names.at(i);
		mark.header.stamp = ros::Time::now();
		mark.ns = "mork";
		mark.id = i;
		mark.type = shape;
		mark.action = visualization_msgs::Marker::ADD;
		mark.pose.position.x = curr_waypoint.at(0);
		mark.pose.position.y = curr_waypoint.at(1);
		mark.pose.position.z = 0;
		mark.pose.orientation.x = 0.0;
		mark.pose.orientation.y = 0.0;
		mark.pose.orientation.z = 0.0;
		mark.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		mark.scale.x = 1.0;
		mark.scale.y = 1.0;
		mark.scale.z = 1.0;

		// Set the color -- be sure to set alpha to something non-zero!
		mark.color.r = 0.0f;
		mark.color.g = 1.0f;
		mark.color.b = 0.0f;
		mark.color.a = 1.0;
		mark.lifetime = ros::Duration();
		marker_pub.publish(mark);
		i++;
		r.sleep();
	}
}

void setup()
{
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	/*Get from param*/
	//robot props
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

	/*Node stuff*/
	marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv,"real_waypoint");
	setup();
	// loop();
	return 0;
}