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
		std::cout << "ctheta: " << tf.ctheta << "\n" << "stheta: " << tf.stheta << "\n"<< "x: " << tf.x << " \n" << "y: " << tf.y << "\n";
		return os;
	}

	//insteam op overload for transforms
	std::istream & operator>>(std::istream & is, Transform2D & tf)
	{
		std::cin >> tf.theta; 
		std::cin >> tf.x;
		std::cin >> tf.y;
		//use as an opprtunity to fill out fields defined by input
		tf.ctheta = cos(tf.theta);
		tf.stheta = sin(tf.theta);
		return is;
	}

	Transform2D Transform2D::inv() const {
		Transform2D inv;
		inv.x = -cos(this->theta) * this->x - sin(this->theta) * this->y;
		inv.y = sin(this->theta) * this->x - cos(this->theta) * this->y;
		inv.ctheta = this->ctheta;
		inv.stheta = -(this->stheta);
		return inv;
	}


}