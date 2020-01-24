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

TEST(Vector2D, add)
{
    Vector2D vec1(1,1);
    Vector2D vec2(2,3);
    Vector2D vec3;
    Vector2D test_vec(3,4);
    vec3 = vec1 + vec2;
    ASSERT_EQ(vec3, test_vec);
    vec1 += vec2;
    ASSERT_EQ(vec1, vec3);
}

TEST(Vector2D, sub)
{
    Vector2D vec1(1,1);
    Vector2D vec2(2,3);
    Vector2D vec3;
    Vector2D test_vec(1,2);
    vec3 = vec2 - vec1;
    ASSERT_EQ(vec3, test_vec);
    vec2 -= vec1;
    ASSERT_EQ(vec2, vec3);
}

TEST(Vector2D, mult)
{
    Vector2D vec1(1,2);
    Vector2D test_vec(5,10);
    double mul = 5;
    Vector2D rhs_mul;
    Vector2D lhs_mul;
    rhs_mul = vec1 * mul;
    lhs_mul = mul * vec1;
    ASSERT_EQ(test_vec, lhs_mul);
    ASSERT_EQ(lhs_mul, rhs_mul);
    vec1 *= mul;
    Vector2D vec2(1,2);
    ASSERT_EQ(vec1, lhs_mul);
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

TEST(Transform2D, constructors)
{
    Transform2D Tab;
    double arr[3];
    Tab.displacement(arr);
    ASSERT_EQ(arr[0],0);
    ASSERT_EQ(arr[1],0);
    ASSERT_EQ(arr[2],0);

    Vector2D vec(2,4);
    Transform2D Tbc(vec);
    Tbc.displacement(arr);
    ASSERT_EQ(arr[0],0);
    ASSERT_EQ(arr[1],2);
    ASSERT_EQ(arr[2],4);

    Transform2D Txy(PI);
    Txy.displacement(arr);
    ASSERT_EQ(arr[0], rad2deg(PI));
    ASSERT_EQ(arr[1],0);
    ASSERT_EQ(arr[2],0);

    Transform2D Tblorb(vec, PI);
    Tblorb.displacement(arr);
    ASSERT_EQ(arr[0],rad2deg(PI));
    ASSERT_EQ(arr[1],2);
    ASSERT_EQ(arr[2],4);

}

TEST(Transform2D, opOverload)
{
    Transform2D Tab;
    std::string input = "90 1 1";
    std::string output = "theta: 90\tctheta: 0\tstheta: 1\nx: 1\ty: 1\n";
    std::stringstream ss_in(input);
    std::stringstream ss_out;
    ss_in >> Tab;
    ss_out << Tab;
    ASSERT_EQ(ss_out.str(), output);
}

TEST(Transform2D, inv)
{
    Vector2D vec0(1,1);
    Vector2D vec1(2,2);
    Transform2D Tab(vec0, PI/2);
    Transform2D Tbc(vec1, 0);
    Transform2D Tba, Tcb, Tac, Tca;
    Tba = Tab.inv();
    Tcb = Tbc.inv();
    Tac = Tab * Tbc;
    Tca = Tac.inv();

    double arr0[3];
    double arr1[3];
    Tac.displacement(arr0);
    ASSERT_DOUBLE_EQ(arr0[0], 90);
    ASSERT_DOUBLE_EQ(arr0[1], -1);
    ASSERT_DOUBLE_EQ(arr0[2], 3);
    Tba.displacement(arr1);
    ASSERT_DOUBLE_EQ(arr1[0], -90);
    ASSERT_DOUBLE_EQ(arr1[1], -1);
    ASSERT_DOUBLE_EQ(arr1[2], 1);
    Tac.inv().displacement(arr0);
    ASSERT_DOUBLE_EQ(arr0[0], -90);
    ASSERT_DOUBLE_EQ(arr0[1], -3);
    ASSERT_DOUBLE_EQ(arr0[2], -1);

}

TEST(Transform2D, parenOpOverload)
{
    Vector2D vec0(1,1);
    Vector2D vec1(2,2);
    Transform2D Tab(vec0, PI/2);
    Transform2D Tbc(vec1, 0);
    Transform2D Tba, Tcb, Tac, Tca;
    Tba = Tab.inv();
    Tcb = Tbc.inv();
    Tac = Tab * Tbc;
    Tca = Tac.inv();
    Vector2D va, vb, vc(3,3);
    va = Tac(vc);
    vb = Tbc(vc);
    ASSERT_DOUBLE_EQ(va.x, -4);
    ASSERT_DOUBLE_EQ(va.y, 6);
    ASSERT_DOUBLE_EQ(vb.x, 5);    
    ASSERT_DOUBLE_EQ(vb.y, 5);    
}

TEST(Transform2D, adjoint)
{
    Vector2D vec0(1,1);
    Vector2D vec1(2,2);
    Transform2D Tab(vec0, PI/2);
    Transform2D Tbc(vec1, 0);
    Transform2D Tba, Tcb, Tac, Tca;
    Tba = Tab.inv();
    Tcb = Tbc.inv();
    Tac = Tab * Tbc;
    Tca = Tac.inv();
    Vector2D va(1,1), vb(4,4), vc(3,3); //va and vb here reflect the expected velocity components of twists produced by the adjoint of Tac & Tbc

    Twist2D Va, Vb, Vc(1,2,2);
    Va = Tac.adjoint(Vc);
    Vb = Tbc.adjoint(Vc);
    ASSERT_EQ(1, Va.omega);
    ASSERT_EQ(1, Vb.omega);
    ASSERT_DOUBLE_EQ(va.x, Va.vel.x);
    ASSERT_DOUBLE_EQ(va.y, Va.vel.y);
    ASSERT_DOUBLE_EQ(vb.x, Vb.vel.x);
    ASSERT_DOUBLE_EQ(vb.y, Vb.vel.y);
}

TEST(Transform2D, intergrateTwist)
{
    Transform2D Twb, Tbbp, Twbp;
    std::string input = "90 -1 3";
    std::stringstream ss(input);
    ss >> Twb;
    Twist2D twist;
    input = " 0 1 1";
    ss.str(std::string());
    ss.clear();
    ss << input;
    ss >> twist;
    Twbp = Twb.intergrateTwist(twist);
    double Twbp_vals[3];
    Twbp.displacement(Twbp_vals);
    ASSERT_EQ(Twbp_vals[0], 90);
    ASSERT_EQ(Twbp_vals[1], -2);
    ASSERT_EQ(Twbp_vals[2], 4);

    //now with nonzero angular vel
    input = "1 1 1";
    ss.str(std::string());
    ss.clear();
    ss << input;
    ss >> twist;
    Twbp = Twb.intergrateTwist(twist);
    Twbp.displacement(Twbp_vals);
    ASSERT_EQ(almost_equal(Twbp_vals[0], 147.296, 1.0e-3), 1);
    ASSERT_EQ(almost_equal(Twbp_vals[1], -2.30117, 1.0e-4), 1);
    ASSERT_EQ(almost_equal(Twbp_vals[2], 3.38177, 1.0e-4), 1); //NOT SURE IF THIS IS THE RIGHT VALUE

}

}