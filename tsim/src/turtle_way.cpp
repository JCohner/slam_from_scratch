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
std::vector<double> x(5,0);
std::vector<double> y(5,0);
std::vector<double> th(5,0);

//define rot and trans vels
const double rot_vel = 0.5; //rad/s
const double trans_vel = 0.5; //m/s

//time trackers
double freq = 60;
ros::Rate r(freq);
unsigned int i; 

void init_telop(){
	//switch off pen and teleport to lower left corner
	turtlesim::SetPen pen_req;
	pen_req.request.off = 1;
	setPen.call(pen_req);

	turtlesim::TeleportAbsolute tele_req;
	// tele_req.request.x = points.at(0).at(0);  
	// tele_req.request.y = points.at(0).at(1);
	// tele_req.request.theta = points.at(0).at(2); 

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
	nh.getParam("/waypoint_x", x);
	nh.getParam("/waypoint_y", y);
	nh.getParam("/waypoint_th", th);
	for (unsigned int i = 0; i < x.size(); i++){
		// points.at(i) = std::vector<int> {x[i], y[i], th[i]};
		waypoints.addWaypoint(std::vector<double> {x[i], y[i], th[i]}, i);
		// ROS_INFO("%d %d",y[i], points.at(i).at(1)); //first .at(waypoint).at(x or y)
	}
	wait_services();
	//iniitalize publishers, services, and subscribers
	setPen = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
	teleportAbsolute = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",5);

	//move turtle to starting pos
	init_telop();
}

void loop(){


	while(ros::ok()){
		// float ellapsed = (i/freq);
		// float trav_time_width = (rect_.width_ / ((float) robo_.trans_vel_));
		// float trav_time_height = (rect_.height_ / ((float) robo_.trans_vel_));
		// float trav_time_rot = ((PI/2.0) / (float) robo_.rot_vel_);
		// switch (state){
		// 	case 0:
		// 		vel_req.linear.x = robo_.trans_vel_;
		// 		vel_req.angular.z = 0;

		// 		// printf("ell: %f \t trav: %f\n", ellapsed, trav_time_width);
		// 		if (ellapsed > trav_time_width){
		// 			state_flag = 1;
		// 			dir_switch = !dir_switch;
		// 		}
		// 		break;
		// 	case 3:
		// 	case 1:
		// 		vel_req.angular.z = robo_.rot_vel_;
		// 		vel_req.linear.x = 0;

		// 		// printf("ell: %f \t trav: %f\n", ellapsed, trav_time_rot);
		// 		if (ellapsed > (trav_time_rot - 1/robo_.frequency_)){
		// 			state_flag = 1;
		// 			// dir_switch = !dir_switch;
		// 			theta_ref_prev = theta_refs[(++theta_refs_index % 4)];
		// 		}
		// 		break;
		// 	case 2:
		// 		vel_req.linear.x = robo_.trans_vel_;
		// 		vel_req.angular.z = 0;

		// 		if (ellapsed > trav_time_height){
		// 			state_flag = 1;
		// 			// dir_switch = !dir_switch;
		// 		}
		// 		break;

		// 	default:
		// 		ROS_INFO("Oh bud you really done goofed\n"); //TODO: replace with std::exception of sorts

		// }
		// ++i;

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