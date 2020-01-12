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

	Transform2D Transform2D::inv() const {
		Transform2D inv;
		inv.x = -cos(deg2rad(this->theta)) * this->x - sin(deg2rad(this->theta)) * this->y;
		inv.y = sin(deg2rad(this->theta)) * this->x - cos(deg2rad(this->theta)) * this->y;
		inv.theta = -(this->theta);
		inv.ctheta = this->ctheta;
		inv.stheta = -(this->stheta);
		return inv;
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

	Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
		Transform2D prod; 
		prod.theta = lhs.theta + rhs.theta;
		prod.ctheta = cos(prod.theta);
		prod.stheta = sin(prod.theta);
		prod.x = cos(lhs.theta) * rhs.x - sin(lhs.theta) * rhs.y + lhs.x;
		prod.y = cos(lhs.theta) * rhs.y + sin(lhs.theta) * rhs.x + lhs.y; 

		return prod;
	}

	Transform2D& operator*=(Transform2D& lhs, const Transform2D & rhs){
		lhs = lhs * rhs;
		return lhs;
	}	

}