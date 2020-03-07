#ifndef CIRC_FIT_INCLUDE_GUARD_HPP
#define CIRC_FIT_INCLUDE_GUARD_HPP
#include <vector>

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

pt find_centroid(Cluster clust);
Cluster shift_center(Cluster clust, pt center);
Circle fit_circle(Cluster clust, pt center);


#endif
