#include "waypoints/waypoints.hpp"

namespace rigid2d
{
	std::vector<double> Waypoints::get_curr_waypoint(){
		return waypoints.at(waypoint_index);
	}

	Twist2D Waypoints::nextWaypoint(){
		Twist2D Vb;
		int curr_state = state;
		int next_state = (state == (waypoints.size() - 1)) ? 0 : state + 1;
		int state_flag;
		double ellapsed = time_it / freq; 
		if (state % 2){
			//turn for odd states
			double rad = waypoints[next_state][2] - waypoints[curr_state][2]; 
			Vb.omega = angular_vel;
			state_flag = state_check(rad, angular_vel, ellapsed);
		} else {
			//straight for odd states
			double dist = sqrt(pow(waypoints[next_state][0] - waypoints[curr_state][0],2) + pow(waypoints[next_state][1] - waypoints[curr_state][1],2));
			Vb.vel.x = trans_vel;
			state_flag = state_check(dist, trans_vel, ellapsed);
		}

		if (state_flag){
			state = (state + 1) % waypoints.size();
			time_it = 0;
			Vb.vel.x = 0;
			Vb.omega = 0;
		} else {
			++time_it;
		}

		return Vb;
	}

	int Waypoints::state_check(double dist, double speed, double ellapsed){
		int flag;
		double trav_time = dist / speed;
		if (ellapsed > trav_time){
			flag = 1;
		} else {
			flag = 0;
		}
		return flag;
	}

	void Waypoints::addWaypoint(std::vector<double> newWaypoint, int index){
		waypoints.at(index) = newWaypoint; 
		return;
	}
}