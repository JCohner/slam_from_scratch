#include "diff_drive/diff_drive.hpp"

namespace rigid2d{
	WheelVelocities DiffDrive::twistToWheels(Twist2D Vb){		
		//sets and returns wheel vel from body twist
		double r = this->wheel_radius;
		double d = this->wheel_base;
		this->wheel_vels.left = 1/r * (-(d)/2 * Vb.omega + Vb.vel.x);
		this->wheel_vels.right = 1/r * ((d)/2 * Vb.omega + Vb.vel.x);
		return this->wheel_vels;
	}


	Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel){
		Twist2D Vb;
		double r = this->wheel_radius;
		double d = this->wheel_base;
		Vb.omega = -r/d * vel.left + r/d * vel.right;
		Vb.vel.x = r/2 * (vel.left + vel.right);

		return Vb;
	}

	void DiffDrive::updateOdometry(double left_rad, double right_rad){
		double r = this->wheel_radius;
		double d = this->wheel_base;
		double left_dist = left_rad * r; //seen as delta ul
		double right_dist = right_rad * r; //seen as delta ur
		Twist2D Vb;
		Vb.omega = r/d * (-left_dist + right_dist);
		Vb.vel.x = r/2 * (left_dist + right_dist);
		this->Twb = (this->Twb).intergrateTwist(Vb);
		this->wheel_vels.left = left_rad;
		this->wheel_vels.right = right_rad;
		return;  
	}

	void DiffDrive::feedforward(Twist2D twist){
		//integrate twist over one unit time step and premultiply by Twb such that you update odom
		//i.e. Twb' = Twb * Tbb' <-- this is what the intergrateTwist function does & what we need
		this->Twb = Twb.intergrateTwist(twist);
		(*this).twistToWheels(twist);
		return;
	}

	Twist2D DiffDrive::pose(){
		/*Important note: using Twist2D to represent theta, x, & y of TF */
		Twist2D pose;
		double Twb_pose[3];
		(this->Twb).displacement(Twb_pose);
		pose.omega = Twb_pose[0];
		pose.vel.x = Twb_pose[1];
		pose.vel.y = Twb_pose[2];
		return pose;
	}

	WheelVelocities DiffDrive::wheelVelocities() const{
		return this->wheel_vels;	
	}

	void DiffDrive::reset(Twist2D ps){
		Transform2D Twb(ps.vel, ps.omega);
		this->Twb = Twb;
		return;
	}

}