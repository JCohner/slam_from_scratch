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
		double angular_vel; //maybe move to diff drive
		double trans_vel; //maybe move to diff drive
		std::vector<std::vector<double>> waypoints;
		unsigned int time_it = 0;
		unsigned int state;
		unsigned int waypoint_index;
		double freq;
		int state_check(double dist, double speed, double ellapsed);
	public:
		Waypoints() : waypoints(5, std::vector<double>(3,0)), time_it(0), state(0), waypoint_index(0){};
		
		Waypoints(Transform2D pose, double angular_vel, double trans_vel, double freq) : 
		robot(pose), angular_vel(angular_vel), trans_vel(trans_vel), waypoints(5, std::vector<double>(3,0)), time_it(0), state(0), waypoint_index(0), freq(freq){};
		
		/// \brief based on current state returns needed body velocity to get to next waypoint
		/// \return a bdy velocity that gets turtlebot to the next waypoint
		Twist2D nextWaypoint();
		void addWaypoint(std::vector<double> newWaypoint, int index);
		std::vector<double> get_curr_waypoint();
		void loop();
	};
}


#endif