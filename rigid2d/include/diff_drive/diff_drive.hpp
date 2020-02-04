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
		double left_count, right_count;
		WheelVelocities wheel_vels;
	public:
	    /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
	    DiffDrive() : Twb(), wheel_base(.2), wheel_radius(.1), left_count(0), right_count(0), wheel_vels() {}; 

	    /// \brief create a DiffDrive model by specifying the pose, and geometry
	    ///
	    /// \param pose - the current position of the robot
	    /// \param wheel_base - the distance between the wheel centers
	    /// \param wheel_radius - the raidus of the wheels
	    DiffDrive(Twist2D pose, double wr, double wb): Twb(pose.vel, pose.omega), wheel_base(wb), wheel_radius(wr), left_count(0), right_count(0), wheel_vels() {};
	    DiffDrive(double wr, double wb): Twb(), wheel_base(wb), wheel_radius(wr), left_count(0), right_count(0), wheel_vels(){};
	    DiffDrive(Transform2D pose): Twb(pose), left_count(0), right_count(0), wheel_vels() {};
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
	    void updateOdometry(double left_rad, double right_rad, double freq);

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

	 	/// \get encoder counts
	    /// \param takes an int counts[2] that it will populate with the left and right encoder counts
	    void get_encoders(double * counts){
	    	counts[0] = left_count;
	    	counts[1] = right_count;
	    	return;
	    }
		
		 /// \set encoder counts
	    /// \param left - left encoder count
	    /// \prarm right - right encoder count
	    void set_encoders(double left, double right){
	    	left_count = left;
	    	right_count = right;
	    }


	};
} 

#endif