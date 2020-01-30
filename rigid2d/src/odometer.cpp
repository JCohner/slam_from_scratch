#include <ros/ros.h>
#include "diff_drive/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

//robot specs
double wheel_base;
double wheel_radius;
double freq;
//this robots listens the encoder updates published on joint states & updates its perception of wheel velocities accordingly
rigid2d::DiffDrive robot;
rigid2d::WheelVelocities w_pos_curr;
rigid2d::WheelVelocities w_pos_prev;
rigid2d::WheelVelocities wheel_vels;

std::string odom;
std::string body;
std::string left_wheel;
std::string right_wheel;

//ros node setup
ros::Subscriber js_sub;
ros::Publisher odom_pub;

//ros time management
ros::Time curr_time, prev_time;

//tf generation variables
double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void publishOdom(){ //rigid2d::Twist2D Vb
	rigid2d::Twist2D Vb = robot.wheelsToTwist(robot.wheelVelocities());
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
	th += delta_th;

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

}

void js_callback(sensor_msgs::JointState data){
	ROS_INFO("%f %f", data.position[0], data.position[1]);
	/*Attempt 0*/
	double encoders[2];
	robot.get_encoders(encoders); //pervious val
	robot.updateOdometry(data.position[0] - encoders[0], data.position[1] - encoders[1], freq);
	robot.set_encoders(data.position[0], data.position[1]);
	publishOdom();
	
	return;
}

void setup(){
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	//get public params
	nh.getParam("wheel/radius", wheel_radius);
	nh.getParam("wheel/base", wheel_base);
	nh.getParam("freq", freq);
	robot.set_wheel_props(wheel_radius, wheel_base);
	//get private params
	nh_priv.getParam("frame_names/odom_frame_id", odom);
	nh_priv.getParam("frame_names/body_frame_id", body);
	nh_priv.getParam("frame_names/left_wheel_joint", left_wheel);
	nh_priv.getParam("frame_names/right_wheel_joint", right_wheel);

	w_pos_prev = rigid2d::WheelVelocities(0,0);

	js_sub = nh.subscribe("/joint_states", 1, &js_callback);
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);


}

void loop(){
	prev_time = ros::Time::now();
	curr_time = ros::Time::now();

	ros::Rate r(freq);
	while(ros::ok()){
		ros::spinOnce();

	}
}


int main(int argc, char * argv[]){
	ros::init(argc, argv,"odometer");
	setup();
	loop();


	return 0;
}
