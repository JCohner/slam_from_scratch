#include <ros/ros.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <tsim/PoseError.h>

#define PI 3.14159

//structures where parameters from parameter server will be stored
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

void wait_services(void){
	//wait for services to become available
	ros::service::waitForService("/turtle1/set_pen");
	ros::service::waitForService("/turtle1/teleport_absolute");
}

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
void compute_error(float t){
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
			if (!dir_switch){
				theta_ref = theta_ref_prev + robo_.rot_vel_ * t;
			}

			switch (theta_refs_index % 4){
				case 0:
					printf("corn 0\n");
					break;
				case 1:
					printf("corn 1\n");
					break;
				case 2:
					printf("corn 2\n");
					break;
				case 3:
					printf("corn 3\n");
					break;
			}

			break;
		case 2:
			if (dir_switch){
				y_ref = robo_.trans_vel_ * t + rect_.y_; 
				x_ref = rect_.x_;
				theta_ref = PI/2.0;
			} else {
				y_ref = rect_.y_ + rect_.height_ - (robo_.trans_vel_ * t); 
				x_ref = rect_.x_ + rect_.width_;
				theta_ref = -PI/2.0;
			}
			break;
		default:
			ROS_INFO("Ya goofed");
	}
}

public:
	TurtleRect()
	{
		get_params();
		wait_services();
		init_telop();
	}
	//declare serviceserver
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

		//switch on pen
		pen_req.request.off = 0;
		setPen_.call(pen_req);
		return true;
		
	}


	void pose_callback(turtlesim::Pose req){
		printf("x: %f \t y: %f\n", req.x, req.y);
		// printf("we here ya\n");
		// ROS_INFO("FUCKKKKK");
	}

	//why doesnt this work thats freaking crazy, it only works if i implement serviceserver and subscriber in my main loop
	// void start_service(void){
	// 	//Define reset service
	// 	ros::ServiceServer reset_service;
	// 	reset_service = nh.advertiseService("/traj_reset", &TurtleRect::reset_callback, this);
		
	// 	// ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 1, &TurtleRect::pose_callback, this);
	// 	// ROS_INFO("Subscriber Listening");
	// }


	
	int loop(){
		ros::ServiceServer reset_service = nh.advertiseService("/traj_reset", &TurtleRect::reset_callback, this);
		ROS_INFO("Service Started");
		ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 1, &TurtleRect::pose_callback, this);
		ROS_INFO("Subscriber Listening");
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
						dir_switch = !dir_switch;
						theta_ref_prev = theta_refs[(++theta_refs_index % 4)];
					}
					break;
				case 2:
					vel_req.linear.x = robo_.trans_vel_;
					vel_req.angular.z = 0;

					if (ellapsed > trav_time_height){
						state_flag = 1;
						dir_switch = !dir_switch;
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

			compute_error(ellapsed);
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
