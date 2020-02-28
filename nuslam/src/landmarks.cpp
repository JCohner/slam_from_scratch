#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nuslam/TurtleMap.h>

ros::Publisher landmark_pub;
ros::Subscriber laser_sub;
bool init_sensor = 0;
double scan_time, range_min, range_max, time_inc, angle_inc, angle_min, angle_max;
int num_data; 
std::vector<double> ranges, intensities;


void laser_sub_callback(sensor_msgs::LaserScan data)
{
	if(!init_sensor)
	{
		scan_time = data.scan_time;
		time_inc = data.time_increment;
		angle_inc = data.angle_increment;
		range_min = data.range_min;
		range_max = data.range_max;
		angle_min = data.angle_min;
		angle_max = data.angle_max;

		num_data = std::round((angle_max - angle_min) /angle_inc);

		ranges.reserve(num_data);
		// intensities.reserve(num_data);
		ROS_INFO("ang min: %f, ang max %f, ang inc: %f, num data %d", angle_min, angle_max, angle_inc, num_data);
		init_sensor = 1;
		
		for (int i = 0; i < num_data; i++)
		{
			ranges.push_back(data.ranges[i]);
		}
	}

	for(int i = 0; i < num_data; i++)
	{
		ranges.at(i) = data.ranges[i];
		ROS_INFO("%f", ranges.at(i));
	}

}

void setup()
{
	ros::NodeHandle nh;
	landmark_pub = nh.advertise<nuslam::TurtleMap>("landmarks", 1);
	laser_sub = nh.subscribe("scan", 1, &laser_sub_callback);
}

int main(int argc, char * argv[]){
	ros::init(argc, argv, "landmarks");
	setup();
	ros::spin();
}