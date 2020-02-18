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
		unsigned int state; //between 0-1 0: trans || 1: rot
		unsigned int waypoint_index; //keep this between 0-4 5 pts on pent
		double freq;
		int state_check(double dist, double speed, double ellapsed);
	public:
		Waypoints() : waypoints(5, std::vector<double>(3,0)), time_it(0), state(0), waypoint_index(0){};
		
		Waypoints(Transform2D pose, double angular_vel, double trans_vel, double freq) : 
			robot(pose), angular_vel(angular_vel), trans_vel(trans_vel), 
			waypoints(5, std::vector<double>(3,0)), 
			time_it(0), state(0), waypoint_index(0), freq(freq){};
		
		/// \brief based on current state returns needed body velocity to get to next waypoint
		/// \return a bdy velocity that gets turtlebot to the next waypoint
		Twist2D nextWaypoint();
		void addWaypoint(std::vector<double> newWaypoint, int index);
		std::vector<double> get_curr_waypoint();
		void loop();

		double get_freq(){
			return freq;
		}
		unsigned int get_state(){
			return state;
		}
		unsigned int get_index(){
			return waypoint_index;
		}

		double get_ang_vel(){
			return angular_vel; //not super effecient buttt
		}
		double get_trans_vel(){
			return trans_vel; //not super effecient buttt
		}
		double get_ellapsed(){
			return (time_it/freq); //not super effecient buttt
		}
		/// \brief prepares waypoint statemachine for begining 
		void start();

		std::vector<double> get_waypoint_at(int index)
		{
			return waypoints.at(index);
		}
	};
}


#endif