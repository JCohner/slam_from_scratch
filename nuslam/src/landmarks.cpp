#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nuslam/TurtleMap.h>
#include <cmath>
#include <Eigen/Dense>

ros::Publisher landmark_pub;
ros::Subscriber laser_sub;
bool init_sensor = 0;
double scan_time, range_min, range_max, time_inc, angle_inc, angle_min, angle_max;
int num_data; 
std::vector<double> ranges;


double dist_thresh = .05;
double clust_num_min= 3; 

struct pt
{
	double x;
	double y;
	bool in_clust;
	int cluster_idx;
	pt() : x(0), y(0), in_clust(false) {}
	pt(double x, double y) : x(x), y(y), in_clust(false) {}
};

struct Cluster{
	int num_pts;
	// std::vector<double> ranges, bearings;
	std::vector<pt> points;
	bool is_viable;
	bool is_circle;
	Cluster (): num_pts(0), is_viable(0), is_circle(0) {}

	void add_to_clust(pt point)
	{
		points.push_back(point);
	}
};

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
	for (int i = 0; i < ranges.size(); i++)
	{
		const double range = ranges.at(i);
		const double x = range * cos(i * angle_inc);
		const double y = range * sin(i * angle_inc);
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
		// ROS_INFO("%f", ranges.at(i));
	}

	detect_clusters();
	int num_via_clusts = viable_clust.size();
	int num_clust = clusters.size();
	ROS_INFO("num clust: %d", num_clust);
	ROS_INFO("num viable clusters: %d", num_via_clusts);

	nuslam::TurtleMap msg;
	int i = 0;

	ros::Rate r(60);
	for (Cluster clust : viable_clust){
		msg.centerX = clust.points.at(0).x;
		msg.centerY = clust.points.at(0).y;
		ROS_INFO("adding point %f, %f", msg.centerX, msg.centerY);
		landmark_pub.publish(msg);
		i++;
		r.sleep();
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