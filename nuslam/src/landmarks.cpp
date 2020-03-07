#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nuslam/TurtleMap.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 

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
	std::vector<pt> points;
	bool is_viable;
	bool is_circle;
	Cluster (): is_viable(0), is_circle(0) {}

	void add_to_clust(pt point)
	{
		points.push_back(point);
	}
};

struct Circle{
	pt center;
	double radius;
	Circle() : center(0,0), radius(0) {}
	Circle(pt point, double radius): center(point), radius(radius) {};
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

	detect_clusters();
	int num_via_clusts = viable_clust.size();
	int num_clust = clusters.size();
	ROS_INFO("num clust: %d", num_clust);
	ROS_INFO("num viable clusters: %d", num_via_clusts);

	// nuslam::TurtleMap msg;
	// int i = 0;

	// // ros::Rate r(60);
	// for (Cluster clust : viable_clust){
	// 	msg.centerX = clust.points.at(0).x;
	// 	msg.centerY = clust.points.at(0).y;
	// 	ROS_INFO("adding point %f, %f", msg.centerX, msg.centerY);
	// 	landmark_pub.publish(msg);
	// 	i++;
	// 	// r.sleep();
	// }

	//circle detect
	double x_cent(0), y_cent(0);
	int num_pts;
	for (Cluster clust : viable_clust)
	{
		//find centroid
		num_pts = clust.points.size();
		for (int i = 0; i < num_pts; i++)
		{
			x_cent += clust.points.at(i).x / (double) num_pts;
			y_cent += clust.points.at(i).y / (double) num_pts;
		}

		//shift points so center is at origin
		for (int i = 0; i < num_pts; i++)
		{
			clust.points.at(i).x -= x_cent; 
			clust.points.at(i).y -= y_cent; 
		}

		//compute mean of distance of points from origin
		//and form data matrix
		double z(0), z_mean(0);
		double x,y;
		Eigen::MatrixXf Z(num_pts, 4);  
		for (int i = 0; i < num_pts; i++)
		{
			x = clust.points.at(i).x;
			y = clust.points.at(i).y;
			z = std::pow(x,2) + std::pow(y,2);
			z_mean += z/num_pts;

			Z(i, 0) = z;
			Z(i, 1) = x;
			Z(i, 2) = y;
			Z(i, 3) = 1;
		}

		Eigen::MatrixXf M(4, 4);
		M = 1/num_pts * Z.transpose() * Z; //chnage build type to debug to make sure we're doing this right

		//make our H matrices
		Eigen::Matrix4f H, H_inv;
		H = H.Zero(4,4);
		H_inv = H_inv.Zero(4,4);
		H(0,0) = 8 * z_mean;
		H(0,3) = 2;
		H(3,0) = 2;
		H(1,1) = 1;
		H(2,2) = 1;

		H_inv(0,3) = 0.5;
		H_inv(3,0) = 0.5;
		H_inv(1,1) = 1;
		H_inv(2,2) = 1;
		H_inv(3,3) = -2 * z_mean;

		// ROS_INFO_STREAM(H);
		// ROS_INFO_STREAM(H_inv);

		//compute SVD of Z
		Eigen::JacobiSVD<Eigen::Matrix4f> svd(Z,  0x04 | 0x10); //values of ComputeFullU | ComputeFullV
		auto U = svd.matrixU();
		auto V = svd.matrixV();
		auto singVals = svd.singularValues();

		double sig4 = singVals(3); //make sure grabbing properly
		ROS_INFO("sing v 4 is: %f", sig4);
		Eigen::Vector4f A;
		Eigen::Matrix4f S = singVals.asDiagonal();
		if (sig4 < 1e-12)
		{
			A = V.col(3);
		} else {
			Eigen::Matrix4f Y = V * S * V.transpose() ;
			Eigen::Matrix4f Q = Y * H_inv * Y;
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> es(Q);
			Eigen::Vector4f eig_vals = es.eigenvalues();

			double min_val(1e10), check_val;// = eig_vals.minCoeff(); //need to find min postiive
			int min_idx = 0;
			for (int i = 0; i < 4; i++)
			{
				check_val = eig_vals(i);
				ROS_INFO("check val is: %f", check_val);
				if (check_val < min_val && check_val > 0) 
				{
					min_idx = i;
					min_val = check_val;
				}
			}			

			ROS_INFO("min john idx: %d", min_idx);
			Eigen::Vector4f A_star = es.eigenvectors().col(min_idx);
			A = Y.transpose() * A_star;
		}

		
		double a = -A(1)/(2 * A(0));
		double b = -A(2)/(2 * A(0));
		double R_squared = (std::pow(A(1),2) + std::pow(A(2),2) - 4 * A(0) * A(3))/(4 * std::pow(A(0),2));
		double R = std::sqrt(R_squared);
		pt center(x_cent + a, y_cent + b);
		Circle circ(center, R);

		//TODO: find root mean square error
		nuslam::TurtleMap msg;
		msg.centerX = circ.center.x;
		msg.centerY = circ.center.y;
		msg.radius = circ.radius;
		landmark_pub.publish(msg);

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