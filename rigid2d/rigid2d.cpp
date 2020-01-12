#include "rigid2d.hpp"

namespace rigid2d
{
	//write constructor
	Transform2D::Transform2D()
	{

	}

	std::ostream & operator<<(std::ostream & os, const Vector2D & v)
	{
		std::cout << "x: " << v.x << "\n";
		std::cout << "y: " << v.y << "\n";
		return os;
	}

	std::istream & operator>>(std::istream & is, Vector2D & v)
	{
		std::cin >> v.x;
		std::cin >> v.y;
		return is;
	}	

	// std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
	// {

	// 	return os;
	// }

	// std::istream & operator>>(std::istream & is, Transform2D & tf)
	// {

	// 	return is;
	// }


}