#include "diff_drive/diff_drive.hpp"

namespace rigid2d{
	WheelVelocities DiffDrive::twistToWheels(Twist2D Vb){
		WheelVelocities w_vels;
		
		w_vels.left = 1/this->wheel_radius * (-(this->wheel_base)/2 * Vb.omega + Vb.vel.x);
		w_vels.right = 1/this->wheel_radius * ((this->wheel_base)/2 * Vb.omega + Vb.vel.x);

		return w_vels;
	}


	Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel){
		Twist2D Vb;
		double r = this->wheel_radius;
		double d = this->wheel_base;
		Vb.omega = -r/d * vel.left + r/d * vel.right;
		Vb.vel.x = r/2 * (vel.left + vel.right);

		return Vb;
	}
}