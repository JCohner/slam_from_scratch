#ifndef WAYPOINTS_INCLUDE_GUARD_HPP
#define WAYPOINTS_INCLUDE_GUARD_HPP

#include <vector>
#include "diff_drive/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

namespace rigid2d
{
	/// \brief calculates velocities for robot to travel between waypoints
	class Waypoints {
		DiffDrive robot;
		double angular_vel = 1;
		double trans_vel = 2;
		unsigned int time_it = 0;
		std::vector<std::vector <double>> waypoints{ {1,1,0}, {1,3, PI/4.0}, {2,4, -PI/4.0}, {3,3, -PI/2}, {3,1, -PI} };
		unsigned int state;
		double freq;
		void init();
		int state_check(double dist, double speed, double ellapsed);
	public:
		Waypoints() : state(0), freq(60)  {init();};
		/// \brief based on current state returns needed body velocity to get to next waypoint
		/// \return a bdy velocity that gets turtlebot to the next waypoint
		Twist2D nextWaypoint();
		void loop();
	};
}


#endif