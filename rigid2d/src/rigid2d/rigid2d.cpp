#include "rigid2d/rigid2d.hpp"

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
	Transform2D::Transform2D(const Vector2D & trans, double radians)
	{
		theta = rad2deg(radians);
		ctheta = cos(radians);
		stheta = sin(radians);
		x = trans.x;
		y = trans.y;
	}

	//aply transform to vector
	Vector2D Transform2D::operator()(Vector2D v) const
	{
		Vector2D v_new;
		v_new.x = this->ctheta * v.x - this->stheta * v.y + this->x;
		v_new.y = this->stheta * v.x + this->ctheta * v.y + this->y;
		return v_new;
	}

	void Transform2D::displacement(double * arr)
	{
            arr[0] = theta;
            arr[1] = x;
            arr[2] = y;
            return;
	}

	Transform2D Transform2D::inv() const 
	{
		Transform2D inv;
		float x_temp = -cos(deg2rad(this->theta)) * this->x - sin(deg2rad(this->theta)) * this->y;
		float y_temp = sin(deg2rad(this->theta)) * this->x - cos(deg2rad(this->theta)) * this->y;
		(almost_equal(x_temp, 0.0)) ? (inv.x = 0) : (inv.x =  x_temp);
		(almost_equal(y_temp, 0.0)) ? (inv.y  = 0) : (inv.y  =  y_temp);

		inv.theta = -(this->theta);
		inv.ctheta = this->ctheta;
		inv.stheta = -(this->stheta);
		return inv;
	}

	Twist2D Transform2D::adjoint(Twist2D V) const
	{
		Twist2D V_prime;
		V_prime.omega = V.omega;
		V_prime.vel.x = cos(deg2rad(this->theta)) * V.vel.x - sin(deg2rad(this->theta)) * V.vel.y + V_prime.omega * this->y;
		V_prime.vel.y = sin(deg2rad(this->theta)) * V.vel.x + cos(deg2rad(this->theta)) * V.vel.y + V_prime.omega * this->x;

		return V_prime;
	}

	double Vector2D::length()
	{
		double len = sqrt(pow(this->x, 2) + pow(this->y, 2));
		return len;
	}

	double Vector2D::distance(const Vector2D & vec){
		//taking this to be the distance between the endpoints of the vectors
		double delta_x = this->x - vec.x;
		double delta_y = this->y - vec.y;
		double dist = sqrt(pow(delta_x, 2) + pow(delta_y,2));
		return dist;
	}

	double Vector2D::angle(){
		double ang = atan2(this->y, this->x);
		return ang;
	}

	//outstream op overload for vectors
	std::ostream & operator<<(std::ostream & os, const Vector2D & v)
	{
		os << "x: " << v.x << "\t";
		os << "y: " << v.y << "\n";
		return os;
	} 

	//instream op overload for vectors
	std::istream & operator>>(std::istream & is, Vector2D & v)
	{
		is >> v.x;
		is >> v.y;
		return is;
	}	

	Vector2D operator+(Vector2D lhs, const Vector2D & rhs){
		Vector2D output;
		output.x = lhs.x + rhs.x;
		output.y = lhs.y + rhs.y;
		return output;
	}

	Vector2D& operator+=(Vector2D& lhs, const Vector2D & rhs){
		lhs = lhs + rhs;
		return lhs;
	}

	Vector2D operator-(Vector2D lhs, const Vector2D & rhs){
		Vector2D output;
		output.x = lhs.x - rhs.x;
		output.y = lhs.y - rhs.y;
		return output;
	}

	Vector2D& operator-=(Vector2D& lhs, const Vector2D & rhs){
		lhs = lhs - rhs;
		return lhs;
	}

	Vector2D operator*(double lhs, const Vector2D & rhs){
		Vector2D vec;
		vec.x = rhs.x * lhs;
		vec.y = rhs.y * lhs;
		return vec;
	}

	/*Ask if we need this implementation*/
	// Vector2D& operator*=(double lhs, Vector2D & rhs){
	// 	rhs = lhs * rhs;
	// 	return rhs;
	// }

	Vector2D operator*(Vector2D lhs, double rhs){
		Vector2D vec;
		vec.x = lhs.x * rhs;
		vec.y = lhs.y * rhs;
		return vec; 
	}

	Vector2D& operator*=(Vector2D& lhs, double rhs){
		lhs = lhs * rhs;
		return lhs;
	}

	bool operator==(const Vector2D & lhs, const Vector2D & rhs)
	{
		return ((lhs.x - rhs.x == 0) && (lhs.y - rhs.y == 0));
	}

	//outstream op overload for transforms2ds
	std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
	{
		os << "theta: " << tf.theta << "\t" <<"ctheta: " << tf.ctheta << "\t" << "stheta: " << tf.stheta << "\n"<< "x: " << tf.x << "\t" << "y: " << tf.y << "\n";
		return os;
	}

	//insteam op overload for transforms
	std::istream & operator>>(std::istream & is, Transform2D & tf)
	{
		is >> tf.theta; 
		is >> tf.x;
		is >> tf.y;
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

	Transform2D Transform2D::intergrateTwist(Twist2D twist){
		Transform2D Trans; //from transform *this aka Twb we will find where input twist takes it aka Twbp
		if (twist.omega != 0){
			Vector2D vel = twist.vel;
			float theta = rad2deg(twist.omega); //+ this->theta; //since we are traversing for one unit time, omega is theta
			int omega = twist.omega/abs(twist.omega); //get the correct sign for unit omega
			float stheta = sin(deg2rad(theta));
			float ctheta = cos(deg2rad(theta));
			Trans.stheta = (almost_equal(stheta,0, 1.0e-6)) ? 0 : stheta; //make sure omega in rad/s
			Trans.ctheta = (almost_equal(ctheta, 0,1.0e-6)) ? 0 : ctheta;
			Trans.theta = rad2deg(atan2(Trans.stheta, Trans.ctheta));
			float x = vel.x * sin(deg2rad(theta)) + vel.y * omega * (cos(deg2rad(theta)) - 1);
			float y = vel.x * omega * (1-cos(deg2rad(theta))) + vel.y * sin(deg2rad(theta));
			Trans.x = x;//+ this->x;
			Trans.y = y; //+ this->y;
			Trans = *this * Trans; //Trans is Tbb' here, premultiply it by Twb to get Twb'
		} else {
			//no rotation case
			Vector2D vel = (*this)(twist.vel); //velocity of input twist in frame Twb
			Trans.theta = this->theta;
			Trans.ctheta = this->ctheta;
			Trans.stheta = this->stheta;
			//new position based on unit timestep of velocity
			Trans.x = vel.x;
			Trans.y = vel.y; //NOT SURE IF THIS IS RIGHT
		}
		// //return the transform wrt to the base frame that started the twist
		// Trans*= *this;
		return Trans;

	}

	//Twist output operator overload
	std::ostream & operator<<(std::ostream & os, const Twist2D & V)
	{
		os << "omega: " << V.omega << "\t" << "v.x: " << V.vel.x << "\t" << "v.y: " << V.vel.y <<"\n";
		return os;
	}

	//Twist input operator overload
	std::istream & operator>>(std::istream & is, Twist2D & V)
	{
		is >> V.omega;
		is >> V.vel.x;
		is >> V.vel.y;
		return is;
	}

	NormVector2D::NormVector2D(){
		mag = 0;
		Vector2D vec; // i would like to learn how to not have to include this line, something to do with new operator
		norm = vec;
	}

}