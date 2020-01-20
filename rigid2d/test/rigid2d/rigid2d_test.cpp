#include <gtest/gtest.h>
#include <ros/ros.h>
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <sstream>

namespace rigid2d {
//Test our constant expressions
TEST(ConstExpr, deg2rad)
{
    ASSERT_DOUBLE_EQ(deg2rad(180.0), PI);
    ASSERT_DOUBLE_EQ(deg2rad(90.0), PI/2);
    ASSERT_DOUBLE_EQ(deg2rad(45), PI/4);   

}
TEST(ConstExpr, rad2deg)
{
    ASSERT_DOUBLE_EQ(rad2deg(PI), 180.0);
    ASSERT_DOUBLE_EQ(rad2deg(2*PI), 360);
    ASSERT_DOUBLE_EQ(rad2deg(PI/4), 45);
    ASSERT_EQ(almost_equal(rad2deg(PI/3), 60), true);  
}
TEST(ConstExpr, almost_equal)
{
    ASSERT_EQ(almost_equal(0, 0), true);
    ASSERT_EQ(almost_equal(0.001, 0.005, 1.0e-2), true);
    ASSERT_EQ(almost_equal(0.001, 0.005, 1.0e-12), false);
}

//test our 2D vectors
TEST(Vector2D, opOverload)
{
    std::string input = "3 2";
    std::string output = "x: 3\ty: 2\n";
    std::stringstream ss_in(input);
    std::stringstream ss_out;

    Vector2D vec;
    ss_in >> vec;
    ss_out << vec;

    ASSERT_EQ(ss_out.str(), output);

}

TEST(Vector2D, constructors)
{
    Vector2D vec;
    ASSERT_EQ(vec.x, 0);
    ASSERT_EQ(vec.y, 0);

    Vector2D vec2(3,2);
    ASSERT_EQ(vec2.x, 3);
    ASSERT_EQ(vec2.y, 2);
}

TEST(Vector2D, length_angle)
{
    Vector2D vec(3,4);
    Vector2D vec2(-5,12);
    ASSERT_EQ(5, vec.length());
    ASSERT_EQ(13, vec2.length());
}

TEST(Vector2D, dist)
{
    Vector2D vec(8,15);
    Vector2D vec1;
    ASSERT_EQ(vec.distance(vec1), 17);
}

TEST(NormVector2D, constructors)
{
    NormVector2D nvec;
    ASSERT_EQ(nvec.get().x , 0);
    ASSERT_EQ(nvec.get().y , 0);
    //this also tests normalization
    NormVector2D nvec2(3, 4);
    ASSERT_EQ(nvec2.get().y , pow(4,2)/pow(5,2));
    ASSERT_EQ(nvec2.get().x , pow(3,2)/pow(5,2));
}

TEST(Twist2D, opOverload)
{
    std::string input = "5 3 2";
    std::string output = "omega: 5\tv.x: 3\tv.y: 2\n";
    std::stringstream ss_in(input);
    std::stringstream ss_out;

    Twist2D twist;
    ss_in >> twist;
    ss_out << twist;

    ASSERT_EQ(ss_out.str(), output);
}

TEST(Transform2D, constructors_iooverloads)
{
    Transform2D Tab;
    std::string input = "90 1 1";
    std::string output = "theta: 90\tctheta: 0\tstheta: 1\nx: 1\ty: 1\n";
    std::stringstream ss_in(input);
    std::stringstream ss_out;
    input = "0 2 2";
    ss_in >> Tab;
    ss_out << Tab;
    ASSERT_EQ(ss_out.str(), output);
}

}