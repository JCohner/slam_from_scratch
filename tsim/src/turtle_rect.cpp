#include <ros/ros.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <geometry_msgs/Twist.h>

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

//define publisher to cmd_vel
ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",5);
public:
	TurtleRect()
	{
		printf("hello wprld\n");
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

		//wait for services to become available
		ros::service::waitForService("/turtle1/set_pen");
		ros::service::waitForService("/turtle1/teleport_absolute");

		//switch off pen and teleport to lower left corner
		turtlesim::SetPen pen_req;
		pen_req.request.off = 1;
		setPen_.call(pen_req);

		turtlesim::TeleportAbsolute tele_req;
		tele_req.request.x = rect_.x_;
		tele_req.request.y = rect_.y_;
		tele_req.request.theta = 0;

		teleportAbsolute_.call(tele_req);

		//start on trajectory
		ros::Rate r(robo_.frequency_);
		int state = 0;
		int state_flag = 0;
		float i = 0;
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
					}
					break;
				case 3:
				case 1:
					vel_req.angular.z = robo_.rot_vel_;
					vel_req.linear.x = 0;

					// printf("ell: %f \t trav: %f\n", ellapsed, trav_time_rot);
					if (ellapsed > trav_time_rot){
						state_flag = 1;
					}
					break;
				case 2:
					vel_req.linear.x = robo_.trans_vel_;
					vel_req.angular.z = 0;

					if (ellapsed > trav_time_height){
						state_flag = 1;
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
			r.sleep();
		}
	}

};


int main(int argc, char * argv[])
{
	ros::init(argc, argv,"hello_world");
	TurtleRect turtle_rect;

}
