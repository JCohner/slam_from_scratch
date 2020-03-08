#include <gtest/gtest.h>
#include "circle_fit/circle_fit.hpp"
#include <iostream>
#include <sstream>

TEST(circle_fit, test1)
{
	std::vector<pt> inputs = {pt(1,7), pt(2,6), pt(5,8), pt(7,7), pt(9,5), pt(3,7)};
	Cluster clust;
	clust.points = inputs;

	pt cent = find_centroid(clust);


	clust = shift_center(clust, cent);

	Circle circ = fit_circle(clust, cent);

	ASSERT_FLOAT_EQ(4.615482, circ.center.x);
	ASSERT_FLOAT_EQ(2.807354, circ.center.y);
	ASSERT_FLOAT_EQ(4.8275752, circ.radius);
}

TEST(circle_fit, test2)
{
	std::vector<pt> inputs = {pt(-1,0), pt(-0.3,-0.06), pt(.3,.1), pt(1,0)};

	Cluster clust;
	clust.points = inputs;

	pt cent = find_centroid(clust);


	clust = shift_center(clust, cent);

	Circle circ = fit_circle(clust, cent);

	ASSERT_NEAR(0.4908357, circ.center.x, 1e-4);
	ASSERT_NEAR(-22.15212, circ.center.y, 1e-4);
	ASSERT_NEAR(22.17979, circ.radius, 1e-4);
}