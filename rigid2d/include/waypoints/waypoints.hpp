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
		std::vector<std::vector <int>> waypoints;
		unsigned int state;
		double freq;
		int state_check(double dist, double speed, double ellapsed);
	public:
		Waypoints() : state(0), freq(60)  {};
		void init();
		/// \brief based on current state returns needed body velocity to get to next waypoint
		/// \return a bdy velocity that gets turtlebot to the next waypoint
		Twist2D nextWaypoint();
		void addWaypoint(std::vector<int> newWaypoint);
		void loop();
	};
}


#endif