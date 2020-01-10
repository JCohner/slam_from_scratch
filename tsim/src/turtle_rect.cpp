/// \file
/// \brief This implements the turtle_rect node, a trajectory generator for a turtlesim_node to follow a rectangle
///
/// PUBLISHES:
///     pose_error (tsim::PoseError): enumerates error from reference trajectory of x,y,theta values
///		turtle1/cmd_vel (geometry_msgs::Twist): publishing commanded velocity such that turtle follows desired trajectory
/// SUBSCRIBES:
///     turtle1/pose (turtlesim::Pose): describes turtle's actual x,y,theta values
/// SERVICES:
///     traj_reset (std_srv::Empty): resets the turtle to the starting corner of trajectory - SERVER
///		CLIENTS: set_pen (turtlesim::SetPen) & teleport_absolute(turtlesim::TeleportAbsolute)

#include <ros/ros.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <tsim/PoseError.h>

#define PI 3.14159

/* Structure Declaration*/
struct rectangle
{
	int x_;
	int y_;
	int width_;
	int height_;
};
struct robot
{
	int trans_vel_;
	int rot_vel_;
	float frequency_;
};
struct corners
{
	int corner_1[2];
	int corner_2[2];
	int corner_3[2];
	int corner_4[2];	
};

/*
Defining our main functional class
*/
class TurtleRect
{
//declatre node handler
ros::NodeHandle nh;

//declare parameter structures
rectangle rect_;
robot robo_;
corners corn_;

//define service clients
ros::ServiceClient setPen_ = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
ros::ServiceClient teleportAbsolute_ = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

//define publisher to; cmd_vel
ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",5);

//function to grab and package main parameters
void get_params(void){
	//get parameters from parameter server
	nh.getParam("/rectangle/x", rect_.x_);
	nh.getParam("/rectangle/y", rect_.y_);
	nh.getParam("/rectangle/width", rect_.width_);
	nh.getParam("/rectangle/height", rect_.height_);
	
	//figure out corners
	corn_.corner_1[0] = rect_.x_;
	corn_.corner_1[1] = rect_.y_;
	corn_.corner_2[0] = rect_.x_ + rect_.width_;
	corn_.corner_2[1] = rect_.y_;
	corn_.corner_3[0] = rect_.x_ + rect_.width_;
	corn_.corner_3[1] = rect_.y_ + rect_.height_;
	corn_.corner_4[0] = rect_.x_;
	corn_.corner_4[1] = rect_.y_ + rect_.height_;

	nh.getParam("/robot/trans_vel", robo_.trans_vel_);
	nh.getParam("/robot/rot_vel", robo_.rot_vel_);
	nh.getParam("/robot/frequency", robo_.frequency_);
}

//wait for turtlesim_node services to become available
void wait_services(void){
	//wait for services to become available
	ros::service::waitForService("/turtle1/set_pen");
	ros::service::waitForService("/turtle1/teleport_absolute");
}

//perform the initial teleportation to the starting corner of the rectangle
void init_telop(){
	//switch off pen and teleport to lower left corner
	turtlesim::SetPen pen_req;
	pen_req.request.off = 1;
	setPen_.call(pen_req);

	turtlesim::TeleportAbsolute tele_req;
	tele_req.request.x = rect_.x_;
	tele_req.request.y = rect_.y_;
	tele_req.request.theta = 0;

	teleportAbsolute_.call(tele_req);

	//switch on pen
	pen_req.request.off = 0;
	setPen_.call(pen_req);
}

/*
Declare and define variables need to keep track of:
_actual : actual values of x,y,theta from subscriber
_ref : reference values of x,y,theta computed from the feedforward model (i.e. current time, velocity, distance to travel || turn for state) 
_error: abs(_ref - _actual)

theta_refs, theta_refs_index, theta_refs_prev, & dir_switch are used to extract extra information from a state machine with only 3 distinct states
*/
int state = 0;
float i = 0;
float x_actual;
float y_actual;
float theta_actual;
float x_error;
float y_error;
float theta_error;
float x_ref;
float y_ref;
float theta_ref;

float theta_refs[4] = {0, PI/2.0, PI, -PI/2.0};
int theta_refs_index = 0;
float theta_ref_prev = 0;
float time;
int dir_switch = 1;

//function to compute error from reference and actual, publishes error to pose_error topic
void compute_error(float t, ros::Publisher pub){
	switch(state){
		case 0:
			if (dir_switch){
				x_ref = robo_.trans_vel_ * t + rect_.x_; 
				y_ref = rect_.y_;
				theta_ref = 0;
			} else {
				x_ref = rect_.x_ + rect_.width_ - (robo_.trans_vel_ * t); 
				y_ref = rect_.y_ + rect_.height_;
				theta_ref = PI;
			}
			break;
		case 3:
		case 1:
			theta_ref = theta_ref_prev + robo_.rot_vel_ * t;

			switch (theta_refs_index % 4){
				case 0:
					x_ref = rect_.x_ + rect_.width_;
					y_ref = rect_.y_;
					break;
				case 1:
					x_ref = rect_.x_ + rect_.width_;
					y_ref = rect_.y_ + rect_.height_;
					break;
				case 2:
					x_ref = rect_.x_;
					y_ref = rect_.y_ + rect_.height_;
					break;
				case 3:
					x_ref = rect_.x_;
					y_ref = rect_.y_;
					break;
			}

			break;
		case 2:
			if (!dir_switch){
				y_ref = robo_.trans_vel_ * t + rect_.y_; 
				x_ref = rect_.x_ + rect_.width_;
				theta_ref = PI/2.0;
			} else {
				y_ref = rect_.y_ + rect_.height_ - (robo_.trans_vel_ * t); 
				x_ref = rect_.x_ ;
				theta_ref = -PI/2.0;
			}
			break;
		default:
			ROS_INFO("Ya goofed");
	}

	x_error = abs(x_ref - x_actual);
	y_error = abs(y_ref - y_actual);
	theta_error = abs(theta_ref - theta_actual);
	// printf("x_e: %f \t y_e: %f \t t_e: %f\n", x_error, y_error, theta_error);
	// printf("x_r: %f \t y_r: %f \t t_r: %f\n", x_ref, y_ref, theta_ref);
	// printf("state: %d \t dir: %d\n", state, dir_switch);

	tsim::PoseError msg;
	msg.x_error = x_error;
	msg.y_error = y_error;
	msg.theta_error = theta_error;
	pub.publish(msg);
}

public:
	//TurtleRect Constructor: gets parameters, waits for services, teleports to start position
	TurtleRect()
	{
		get_params();
		wait_services();
		init_telop();
	}

	//Reset Service Callback: /traj_reset "{}"
	bool reset_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
		printf("request heard\n");
		
		turtlesim::SetPen pen_req;
		pen_req.request.off = 1;
		setPen_.call(pen_req);


		turtlesim::TeleportAbsolute tele_req;
		tele_req.request.x = rect_.x_;
		tele_req.request.y = rect_.y_;
		this->teleportAbsolute_.call(tele_req);
		state = 0;
		i = 0;
		theta_refs_index = 0;
		dir_switch = 1;

		//switch on pen
		pen_req.request.off = 0;
		setPen_.call(pen_req);
		return true;
		
	}

	//Pose subscriber callback, cache's actual position and theta
	void pose_callback(turtlesim::Pose req){
		// printf("x: %f \t y: %f\n", req.x, req.y);
		x_actual = req.x;
		y_actual = req.y;
		theta_actual = req.theta;
	}

	/*
	FOR MATT: Subscribers and Service Servers defined here do not work with the structure of my code. I wasted more hours than I would like to admit on this
	Why does it not work? How does spinOnce and threads factor into this? It would be a shame if I could not learn from this mistake.
	 */
	// void start_service(void){
	// 	//Define reset service
	// 	ros::ServiceServer reset_service;
	// 	reset_service = nh.advertiseService("/traj_reset", &TurtleRect::reset_callback, this);
		
	// 	// ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 1, &TurtleRect::pose_callback, this);
	// 	// ROS_INFO("Subscriber Listening");
	// }

	//Main Loop Function	
	int loop(){
		ros::ServiceServer reset_service = nh.advertiseService("/traj_reset", &TurtleRect::reset_callback, this);
		ROS_INFO("Service Started");
		ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 1, &TurtleRect::pose_callback, this);
		ROS_INFO("Subscriber Listening");
		ros::Publisher pos_err_pub = nh.advertise<tsim::PoseError>("pose_error", 5);
		ROS_INFO("Publisher Defined");

		//start on trajectory
		ros::Rate r(robo_.frequency_);
		int state_flag = 0;
		geometry_msgs::Twist vel_req;
		//command velocity based on state
		while (ros::ok()){
			float ellapsed = (i/((float)robo_.frequency_));
			float trav_time_width = (rect_.width_ / ((float) robo_.trans_vel_));
			float trav_time_height = (rect_.height_ / ((float) robo_.trans_vel_));
			float trav_time_rot = ((PI/2.0) / (float) robo_.rot_vel_);
			switch (state){
				case 0:
					vel_req.linear.x = robo_.trans_vel_;
					vel_req.angular.z = 0;

					// printf("ell: %f \t trav: %f\n", ellapsed, trav_time_width);
					if (ellapsed > trav_time_width){
						state_flag = 1;
						dir_switch = !dir_switch;
					}
					break;
				case 3:
				case 1:
					vel_req.angular.z = robo_.rot_vel_;
					vel_req.linear.x = 0;

					// printf("ell: %f \t trav: %f\n", ellapsed, trav_time_rot);
					if (ellapsed > (trav_time_rot - 1/robo_.frequency_)){
						state_flag = 1;
						// dir_switch = !dir_switch;
						theta_ref_prev = theta_refs[(++theta_refs_index % 4)];
					}
					break;
				case 2:
					vel_req.linear.x = robo_.trans_vel_;
					vel_req.angular.z = 0;

					if (ellapsed > trav_time_height){
						state_flag = 1;
						// dir_switch = !dir_switch;
					}
					break;

				default:
					ROS_INFO("Oh bud you really done goofed\n");

			}
			++i;
			if (state_flag){
				state = (state + 1) % 4;
				state_flag = 0;
				i = 0;
			}
			vel_pub.publish(vel_req);

			compute_error(ellapsed, pos_err_pub);
			r.sleep();
			ros::spinOnce();
		}
	}

};


int main(int argc, char * argv[])
{
	ros::init(argc, argv,"turtle_rect");
	TurtleRect turtle_rect;
	// turtle_rect.start_service() //see my comment at the member function declaration. i have no idea why its behaving like this
	turtle_rect.loop();

}
