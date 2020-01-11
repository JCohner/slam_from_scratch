#include "rigid2d.hpp"

namespace rigid2d
{
	Transform2D::Transform2D()
	{

	}

	std::ostream & operator<<(std::ostream & os, const Transform2D & tf){


		return os;
	}

	std::istream & operator>>(std::istream & is, Transform2D & tf){


		return is;
	}


}