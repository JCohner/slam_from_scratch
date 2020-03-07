#include <cmath>
#include <Eigen/Eigenvalues> 
#include <Eigen/Dense>
#include "circle_fit/circle_fit.hpp"

pt find_centroid(Cluster clust)
{
	//find centroid
	int num_pts = clust.points.size();
	double x_cent(0), y_cent(0);
	for (int i = 0; i < num_pts; i++)
	{
		x_cent += clust.points.at(i).x / (double) num_pts;
		y_cent += clust.points.at(i).y / (double) num_pts;
	}

	return pt(x_cent, y_cent);
}

Cluster shift_center(Cluster clust, pt center)
{
	double x_cent = center.x;
	double y_cent = center.y;
	int num_pts = clust.points.size();

	//shift points so center is at origin
	for (int i = 0; i < num_pts; i++)
	{
		clust.points.at(i).x -= x_cent; 
		clust.points.at(i).y -= y_cent; 
	}

	return clust;
}

Circle fit_circle(Cluster clust, pt center)
{
	int num_pts = clust.points.size();

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
	M = 1.0/((double)num_pts) * Z.transpose() * Z; //chnage build type to debug to make sure we're doing this right

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
	// ROS_INFO("sing v 4 is: %f", sig4);
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
			// ROS_INFO("check val is: %f", check_val);
			if (check_val < min_val && check_val > 0) 
			{
				min_idx = i;
				min_val = check_val;
			}
		}			

		// ROS_INFO("min john idx: %d", min_idx);
		Eigen::Vector4f A_star = es.eigenvectors().col(min_idx);
		A = Y.inverse() * A_star;
	}

		
		double a = -A(1)/(2 * A(0));
		double b = -A(2)/(2 * A(0));
		double R_squared = (std::pow(A(1),2) + std::pow(A(2),2) - 4 * A(0) * A(3))/(4 * std::pow(A(0),2));
		double R = std::sqrt(R_squared);
		pt cent(center.x + a, center.y + b);
		return Circle(cent, R);	
}