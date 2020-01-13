#include "rigid2d.hpp"

namespace rigid2d
{
	//write constructor
	Transform2D::Transform2D()
	{
		theta = 0;
		ctheta = 1;
		stheta = 0;
		x = 0; 
		y = 0;
	}

	//pure translation constructor
	Transform2D::Transform2D(const Vector2D & trans){
		theta = 0;
		ctheta = 0;
		stheta = 0;
		x = trans.x;
		y = trans.y;
	}

	//pure rotation constructor
	Transform2D::Transform2D(double radians){
		theta = rad2deg(radians);
		ctheta = cos(radians);
		stheta = sin(radians);
		x = 0;
		y = 0;
	}

	//combo constructor
	Transform2D::Transform2D(const Vector2D & trans, double radians){
		theta = rad2deg(radians);
		ctheta = cos(radians);
		stheta = sin(radians);
		x = trans.x;
		y = trans.y;
	}

	//aply transform to vector
	Vector2D Transform2D::operator()(Vector2D v) const{
		Vector2D v_new;
		v_new.x = this->ctheta * v.x - this->stheta * v.y + this->x;
		v_new.y = this->stheta * v.x + this->ctheta * v.y + this->y;
		return v_new;
	}

	Transform2D Transform2D::inv() const {
		Transform2D inv;
		inv.x = -cos(deg2rad(this->theta)) * this->x - sin(deg2rad(this->theta)) * this->y;
		inv.y = sin(deg2rad(this->theta)) * this->x - cos(deg2rad(this->theta)) * this->y;
		inv.theta = -(this->theta);
		inv.ctheta = this->ctheta;
		inv.stheta = -(this->stheta);
		return inv;
	}

	Twist2D Transform2D::adjoint(Twist2D V) const{
		Twist2D V_prime;
		V_prime.omega = V.omega;
		V_prime.vel.x = cos(deg2rad(this->theta)) * V.vel.x - sin(deg2rad(this->theta)) * V.vel.y;
		V_prime.vel.x = sin(deg2rad(this->theta)) * V.vel.x + cos(deg2rad(this->theta)) * V.vel.y;

		return V_prime;
	}

	//outstream op overload for vectors
	std::ostream & operator<<(std::ostream & os, const Vector2D & v)
	{
		std::cout << "x: " << v.x << "\n";
		std::cout << "y: " << v.y << "\n";
		return os;
	} 

	//instream op overload for vectors
	std::istream & operator>>(std::istream & is, Vector2D & v)
	{
		std::cin >> v.x;
		std::cin >> v.y;
		return is;
	}	

	//outstream op overload for transforms2ds
	std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
	{
		std::cout << "theta: " << tf.theta << "\t" <<"ctheta: " << tf.ctheta << "\t" << "stheta: " << tf.stheta << "\n"<< "x: " << tf.x << " \t" << "y: " << tf.y << "\n";
		return os;
	}

	//insteam op overload for transforms
	std::istream & operator>>(std::istream & is, Transform2D & tf)
	{
		std::cin >> tf.theta; 
		std::cin >> tf.x;
		std::cin >> tf.y;
		//use as an opprtunity to fill out fields defined by input
		float x = cos(deg2rad(tf.theta));
		float y = sin(deg2rad(tf.theta));
		(almost_equal(x, 0.0)) ? (tf.ctheta = 0) : (tf.ctheta =  x);
		(almost_equal(y, 0.0)) ? (tf.stheta = 0) : (tf.stheta =  y);

		return is;
	}

	Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
	{
		Transform2D prod; 
		prod.theta = lhs.theta + rhs.theta;
		prod.ctheta = cos(deg2rad(prod.theta));
		prod.stheta = sin(deg2rad(prod.theta));
		prod.x = cos(deg2rad(lhs.theta)) * rhs.x - sin(deg2rad(lhs.theta)) * rhs.y + lhs.x;
		prod.y = cos(deg2rad(lhs.theta)) * rhs.y + sin(deg2rad(lhs.theta)) * rhs.x + lhs.y; 

		return prod;
	}

	Transform2D& operator*=(Transform2D& lhs, const Transform2D & rhs)
	{
		lhs = lhs * rhs;
		return lhs;
	}	

	//Twist output operator overload
	std::ostream & operator<<(std::ostream & os, const Twist2D & V)
	{
		std::cout << "omega: " << V.omega << "\n" << "v.x: " << V.vel.x << "\t" << "v.y: " << V.vel.y <<"\n";
		return os;
	}

	//Twist input operator overload
	std::istream & operator>>(std::istream & is, Twist2D & V)
	{
		std::cin >> V.omega;
		std::cin >> V.vel.x;
		std::cin >> V.vel.y;
		return is;
	}


}