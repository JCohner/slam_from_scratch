/// \file
/// \brief This implements the odometer node. Which takes encoder reading from joint state and converts those into wheel then body twists. Those body twists are used to publish odometry transforms
///
/// PUBLISHES:
///     odom: <nav_msgs::Odometry>: creates odom to base transfrom 
/// SUBSCRIBES:
///     joint_states: <sensor_msgs::JointState>: fake encoder values
/// SERVICES:
///     set pose: <turtlesim::TeleportAbsolute>: fake encoder values
#include <ros/ros.h>
#include "diff_drive/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <turtlesim/TeleportAbsolute.h> //if youre running this on a turtlebot without turtlesim installed just make your own srv (lazy)
#include <std_srvs/Empty.h>

//robot specs
static double wheel_base;
static double wheel_radius;
static double encoder_ticks_per_rev;
static double freq;
//this robots listens the encoder updates published on joint states & updates its perception of wheel velocities accordingly
static rigid2d::DiffDrive robot;
static rigid2d::WheelVelocities w_pos_curr;
static rigid2d::WheelVelocities w_pos_prev;
static rigid2d::WheelVelocities wheel_vels;

static std::string odom;
static std::string body;
static std::string left_wheel;
static std::string right_wheel;

//ros node setup
static ros::Subscriber js_sub;
static ros::Publisher odom_pub;
static ros::ServiceServer set_pose;

//ros time management
static ros::Time curr_time, prev_time;

//tf generation variables
static double x = 0.0;
static  double y = 0.0;
static double th = 0.0;

static double vx = 0.0;
static double vy = 0.0;
static double vth = 0.0;

void publishOdom(){ //rigid2d::Twist2D Vb
	rigid2d::Twist2D Vb = robot.wheelsToTwist(robot.wheelVelocities());
	// rigid2d::Twist2D pose = robot.pose();
	// x = pose.vel.x;
	// y = pose.vel.y;
	// th = rigid2d::deg2rad(pose.omega); 

	// ROS_INFO("achieved body twist of: omega: %f, vx: %f, vy: %f", Vb.omega, Vb.vel.x, Vb.vel.y);
	vy = Vb.vel.y;
	vx = Vb.vel.x;
	vth = Vb.omega; 

	static tf2_ros::TransformBroadcaster odom_broadcaster;
	curr_time = ros::Time::now();
	double dt = (curr_time - prev_time).toSec();
	double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th; //correct right hand rule notation 
	// ROS_INFO(")DOM at x: %f, y: %f, th: %f", x, y, th);



	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//publish transform over tf using broadcaster
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = curr_time;
	odom_trans.header.frame_id = odom;
	odom_trans.child_frame_id = body;
	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send transform
	odom_broadcaster.sendTransform(odom_trans);

	//set up odom message to be published over ROS this will make the transform between our baselink and the doom frame
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = curr_time;
	odom_msg.header.frame_id = odom;

	//set pos
	odom_msg.pose.pose.position.x = x;
	odom_msg.pose.pose.position.y = y;
	odom_msg.pose.pose.position.z = 0.0;
	odom_msg.pose.pose.orientation = odom_quat;

	//set vel
	odom_msg.child_frame_id = body;
	odom_msg.twist.twist.linear.x = vx;
	odom_msg.twist.twist.linear.y = vy;
	odom_msg.twist.twist.angular.z = vth;

	//publish odom message
	odom_pub.publish(odom_msg);
	prev_time = curr_time;
}

void js_callback(sensor_msgs::JointState data){
	double encoders[2];
	robot.get_encoders(encoders); //pervious val

	robot.updateOdometry(data.position[0] - encoders[0], data.position[1] - encoders[1], freq);
	// robot.updateOdometry(data.position[0] - encoders[0], data.position[1] - encoders[1], 1);

	// rigid2d::Twist2D pose = robot.pose();
	// ROS_INFO("im at x: %f, y: %f, th: %f", pose.vel.x, pose.vel.y, pose.omega);
	
	robot.set_encoders(data.position[0], data.position[1]);
	publishOdom();
	
	return;
}


bool set_pose_callback(turtlesim::TeleportAbsolute::Request& request, turtlesim::TeleportAbsolute::Response& response){
	robot.reset(rigid2d::Twist2D(request.theta,request.x,request.y));
	rigid2d::Twist2D turt_pose = robot.pose();
	x = turt_pose.vel.x;
	y = turt_pose.vel.y;
	th = rigid2d::deg2rad(turt_pose.omega);
	// ROS_INFO("Turtle placed at: %f, %f, %f", x, y, th);
	// publishOdom();
	return true;
}

void setup(){
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	//get public params
	nh.getParam("/wheel/radius", wheel_radius);
	nh.getParam("/wheel/base", wheel_base);
	nh.getParam("/freq", freq);
	nh.getParam("/wheel/encoder_ticks_per_rev", encoder_ticks_per_rev);
	robot.set_wheel_props(wheel_radius, wheel_base);
	robot.reset(rigid2d::Twist2D(0, 0, 0));
	rigid2d::Twist2D pose = robot.pose();
	x = pose.vel.x;
	y = pose.vel.y;
	th = rigid2d::deg2rad(pose.omega);
	//get private params
	nh_priv.getParam("frame_names/odom_frame_id", odom);
	nh_priv.getParam("frame_names/body_frame_id", body);
	nh_priv.getParam("frame_names/left_wheel_joint", left_wheel);
	nh_priv.getParam("frame_names/right_wheel_joint", right_wheel);

	js_sub = nh.subscribe("joint_states", 1, &js_callback);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
	set_pose = nh.advertiseService("set_pose", &set_pose_callback);
}

void loop(){
	prev_time = ros::Time::now();
	curr_time = ros::Time::now();

	ros::Rate r(freq);
	while(ros::ok()){
		ros::spinOnce();
		r.sleep();
	}
}


int main(int argc, char * argv[]){
	ros::init(argc, argv,"odometer");
	setup();
	loop();


	return 0;
}
