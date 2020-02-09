#include <gtest/gtest.h>
#include <ros/ros.h>
#include "rigid2d/rigid2d.hpp"
#include "diff_drive/diff_drive.hpp"
#include <iostream>
#include <sstream>

namespace rigid2d {
	TEST(DiffDrive, constructor)
	{
		DiffDrive driver;
		WheelVelocities vels = driver.wheelVelocities();
		Twist2D pose = driver.pose();
		ASSERT_EQ(vels.left, 0);
		ASSERT_EQ(vels.right, 0);
		ASSERT_EQ(pose.omega, 0);
		ASSERT_EQ(pose.vel.x, 0);
		ASSERT_EQ(pose.vel.y, 0);
		Twist2D newpose(3,2,1);
		DiffDrive driva(newpose, 0.2, 0.4);
		pose = driva.pose();
		ASSERT_EQ(pose.vel, newpose.vel);
		ASSERT_EQ(pose.omega, rad2deg(newpose.omega));
	}

	TEST(DiffDrive, twistToWheels)
	{
		Twist2D Vb(PI/4, 0, 0);
		DiffDrive driver(0.02, 1.0);
		WheelVelocities vels = driver.twistToWheels(Vb);
		ASSERT_NEAR(vels.left, -19.6349, 1.0e-4);
		ASSERT_NEAR(vels.right, 19.6349, 1.0e-4);

		Vb.vel.x = 1;
		vels = driver.twistToWheels(Vb);
		ASSERT_NEAR(vels.left, 30.36504, 1.0e-4);
		ASSERT_NEAR(vels.right, 69.6349, 1.0e-4);
	}

	TEST(DiffDrive,wheelsToTwist)
	{
		WheelVelocities vels(-19.6349, 19.6349);
		DiffDrive driver(0.02, 1.0);
		Twist2D twist = driver.wheelsToTwist(vels);
		ASSERT_NEAR(twist.omega, PI/4, 1.0e-4);
		ASSERT_EQ(twist.vel.x, 0);
		ASSERT_EQ(twist.vel.y, 0);
		vels.left = 30.36504;
		vels.right = 69.6349;
		twist = driver.wheelsToTwist(vels);
		ASSERT_NEAR(twist.omega, PI/4, 1.0e-4);
		ASSERT_NEAR(twist.vel.x, 1,1.0e-4);
		ASSERT_EQ(twist.vel.y, 0);

	}
	TEST(DiffDrive, updateOdometry)
	{
		DiffDrive diff_drive(0.2, 1.0);
		diff_drive.updateOdometry(10.0,10.0,1);
		Twist2D pose;
		pose = diff_drive.pose();
		ASSERT_EQ(pose.omega, 0); //theta should remain zero
		ASSERT_EQ(pose.vel.x, 2);
		ASSERT_EQ(pose.vel.y, 0);
	}
	TEST(DiffDrive, feedforward)
	{
		DiffDrive diff_drive(0.2, 1.0);
		Twist2D Vb(PI/2, 5, 0);
		diff_drive.feedforward(Vb);
		Twist2D pos = diff_drive.pose();
		ASSERT_EQ(deg2rad(pos.omega), PI/2);
		ASSERT_EQ(pos.vel.y, 5);
		ASSERT_EQ(pos.vel.x, 5);
	}
	TEST(DiffDrive, pose)
	{

	}
	TEST(DiffDrive, wheelVelocities)
	{

	}
	TEST(DiffDrive, reset)
	{

	}
}