#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include "rigid2d/rigid2d.hpp"
#include <exception>

namespace rigid2d
{
	struct WheelVelocities {
		WheelVelocities(): left(0), right(0){};
		WheelVelocities(double left, double right) : left(left), right(right) {};
		double left; //when looking from behind
		double right;
	};

	class DiffDrive
	{
	private:
		Transform2D Twb;
		double wheel_base, wheel_radius;
		WheelVelocities wheel_vels;
	public:
	    /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
	    DiffDrive() : Twb(), wheel_base(.2), wheel_radius(.1), wheel_vels() {}; 

	    /// \brief create a DiffDrive model by specifying the pose, and geometry
	    ///
	    /// \param pose - the current position of the robot
	    /// \param wheel_base - the distance between the wheel centers
	    /// \param wheel_radius - the raidus of the wheels
	    DiffDrive(Twist2D pose, double wr, double wb): Twb(pose.vel, pose.omega), wheel_base(wb), wheel_radius(wr), wheel_vels() {};
	    DiffDrive(double wr, double wb): Twb(), wheel_base(wb), wheel_radius(wr) {};
	    DiffDrive(Transform2D pose): Twb(pose) {};
	    /// \brief determine the wheel velocities required to make the robot
	    ///        move with the desired linear and angular velocities
	    /// \param twist - the desired twist in the body frame of the robot
	    /// \returns - the wheel velocities to use
	    /// \throws std::exception
	    WheelVelocities twistToWheels(Twist2D Vb);

	    /// \brief determine the body twist of the robot from its wheel velocities
	    /// \param vel - the velocities of the wheels, assumed to be held constant
	    ///  for one time unit
	    /// \returns twist in the original body frame of the
	    Twist2D wheelsToTwist(WheelVelocities vel);

	    /// \brief Update the robot's odometry based on the current encoder readings
	    /// \param left - the left encoder angle (in radians)
	    /// \param right - the right encoder angle (in radians)
	    void updateOdometry(double left_rad, double right_rad);

	    /// \brief update the odometry of the diff drive robot, assuming that
	    /// it follows the given body twist for one time  unit
	    /// \param cmd - the twist command to send to the robot
	    void feedforward(Twist2D twist);

	    /// \brief get the current pose of the robot
	    Twist2D pose();

	    /// \brief get the wheel speeds, based on the last encoder update
	    /// \returns the velocity of the wheels, which is equivalent to
	    /// displacement because \Delta T = 1
	    WheelVelocities wheelVelocities() const;

	    /// \brief reset the robot to the given position/orientation
	    void reset(Twist2D ps);

	    /// \set wheel variables after creation
	    /// \param wr - wheel radius in m
	    /// \prarm wb - wheel base in m
	    void set_wheel_props(double wr, double wb);

	};
} 

#endif