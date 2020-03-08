#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nuslam/TurtleMap.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 

#include "circle_fit/circle_fit.hpp"

ros::Publisher landmark_pub;
ros::Subscriber laser_sub;
bool init_sensor = 0;
double scan_time, range_min, range_max, time_inc, angle_inc, angle_min, angle_max;
int num_data; 
std::vector<double> ranges;


double dist_thresh = .05 ;
double radius_thresh = .02;
double clust_num_min= 3; 

std::vector<Cluster> clusters;
std::vector<Cluster> viable_clust;

double pt_distance(pt p1, pt p2)
{
	return std::sqrt(std::pow((p1.x - p2.x),2) + std::pow((p1.y - p2.y),2));
}

//TODO: figure out how to wrap cluster detection
void detect_clusters()
{
	clusters.clear();
	viable_clust.clear();
	pt prev_pt; 
	for (int i = 0; i < (int)ranges.size(); i++)
	{
		const double range = ranges.at(i);
		const double x = range * cos(i * angle_inc);
		const double y = range * sin(i * angle_inc);
		if (i == 0)
		{
			double f_range = ranges.at(ranges.size() - 1);
			prev_pt = pt(f_range * cos(-1 * angle_inc), f_range * sin(-1 * angle_inc));
		}

		pt curr_pt(x,y);
		// ROS_INFO("pt at: %f, %f", x, y);
		// ROS_INFO("prev pt at: %f, %f", prev_pt.x, prev_pt.y);
		const double dist = pt_distance(curr_pt,prev_pt);
		// ROS_INFO("dist between points is %f", dist);
		if (dist < dist_thresh)
		{	
			// ROS_INFO("pts within thresh, prev pt in clust is : %d ", prev_pt.in_clust);
			Cluster clust;
			curr_pt.in_clust = true;
			//if previous pt was in cluster
			if (prev_pt.in_clust)
			{
				clust = clusters.at(prev_pt.cluster_idx);
				// ROS_INFO("previous pt in clust: %d of siz: %d", prev_pt.cluster_idx, (int)clust.points.size());
				
				clust.add_to_clust(curr_pt);
				if ((clust.points.size() > 3) && !(clust.is_viable))
				{
					// ROS_INFO("new viable clust maed!");
					clust.is_viable = true;
					viable_clust.push_back(clust);
					// ROS_INFO("num via clusts: %d",(int) viable_clust.size());
				}

				clusters.at(prev_pt.cluster_idx) = clust;
			}
			//make a new cluster
			else 
			{
				// ROS_INFO("new clust");
				clust.add_to_clust(prev_pt);
				clust.add_to_clust(curr_pt);
				clusters.push_back(clust);
				curr_pt.cluster_idx = clusters.size() - 1;
			}
		}

		prev_pt = curr_pt;
	}
}

void sesnor_init(sensor_msgs::LaserScan data)
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


void get_sensor_reading(sensor_msgs::LaserScan data)
{
	float val;
	for(int i = 0; i < num_data; i++)
	{
		val = data.ranges[i];
		if (val < range_max && val > range_min){
			ranges.at(i) = val;
		} else if (val > range_max) {
			ranges.at(i) = std::numeric_limits<double>::infinity();
		} else {
			ranges.at(i) = 0;
		}
		// ROS_INFO("%f", ranges.at(i));
	}

}

void circle_detect()
{

	//circle detect
	// double x_cent(0), y_cent(0);
	// int num_pts;
	for (Cluster clust : viable_clust)
	{
		pt cent = find_centroid(clust);

		clust = shift_center(clust, cent);

		Circle circ = fit_circle(clust, cent);
		

		ROS_INFO("detecting radius of: %f", circ.radius);

		// if (R > (0.04 - radius_thresh) && R < (0.04 + radius_thresh))
		if(circ.radius < 0.1)
		{
			//TODO: find root mean square error
			nuslam::TurtleMap msg;
			msg.centerX = circ.center.x;
			msg.centerY = circ.center.y;
			msg.radius = circ.radius;
			landmark_pub.publish(msg);
		}

	}
}

void laser_sub_callback(sensor_msgs::LaserScan data)
{
	//if sensor hasn't been initialized, initialize
	if(!init_sensor)
	{
		sesnor_init(data);
	}

	//populate range vector with range values
	get_sensor_reading(data);

	//detect clusters amongst ranges
	detect_clusters();

	int num_via_clusts = viable_clust.size();
	int num_clust = clusters.size();
	ROS_INFO("num clust: %d", num_clust);
	ROS_INFO("num viable clusters: %d", num_via_clusts);

	nuslam::TurtleMap msg;
	int i = 0;

	// ros::Rate r(60);
	for (Cluster clust : viable_clust){
		msg.centerX = clust.points.at(0).x;
		msg.centerY = clust.points.at(0).y;
		msg.clustOcirc = 1;
		ROS_INFO("adding point %f, %f", msg.centerX, msg.centerY);
		landmark_pub.publish(msg);
		i++;
		// r.sleep();
	}

	circle_detect();

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