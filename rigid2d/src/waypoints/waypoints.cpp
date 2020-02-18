#include "waypoints/waypoints.hpp"

namespace rigid2d
{
	std::vector<double> Waypoints::get_curr_waypoint(){
		return waypoints.at(waypoint_index);
	}

	Twist2D Waypoints::nextWaypoint(){
		Twist2D Vb;
		int curr_waypoint = waypoint_index;
		int prev_waypoint = (waypoint_index == 0) ? (waypoints.size() - 1) : waypoint_index - 1;
		int state_flag;
		double ellapsed = time_it / freq; 
		// printf("curr waypoint: %d\n", curr_waypoint);
		// printf("curr state: %d\n", state);
		if (state % 2){
			//turn for odd states
			double rad = waypoints.at(curr_waypoint).at(2); 
			rad = normalize_angle(rad);
			// printf("rad: %f\n", rad);
			Vb.omega = angular_vel;
			// printf("rot_speed: %f\n", angular_vel);		
			state_flag = state_check(rad, angular_vel, ellapsed);
		} else {
			//straight for odd states
			double dist = sqrt(pow(waypoints.at(prev_waypoint).at(0)  - waypoints.at(curr_waypoint).at(0),2) + pow(waypoints.at(prev_waypoint).at(1) - waypoints.at(curr_waypoint).at(1),2));
			// printf("dist: %f\n", dist);
			Vb.vel.x = trans_vel;
			// printf("speed: %f\n", trans_vel);
			state_flag = state_check(dist, trans_vel, ellapsed);
		}

		if (state_flag){
			// printf("CHANGING STATE\tCHANGING STATE\tCHANGING STATE\n");
			state = (state + 1) % 2;
			if(!state){
				waypoint_index = (waypoint_index+1) % 5;
			}
			time_it = 0;
			Vb.vel.x = 0;
			Vb.omega = 0;
		} else {
			++time_it;
			// printf("time_it: %u\n", time_it);
			// printf("ellapsed: %f\n", ellapsed);
		}

		return Vb;
	}

	int Waypoints::state_check(double dist, double speed, double ellapsed){
		double trav_time = fabs(dist) / speed;
		// std::cout << "expected trav time: " << trav_time << "\n";
		if (ellapsed > trav_time){
			return 1;
		} else {
			return 0;
		}
	}

	void Waypoints::addWaypoint(std::vector<double> newWaypoint, int index){
		waypoints.at(index) = newWaypoint; 
		return;
	}

	void Waypoints::start(){
		++waypoint_index;
		return;
	}
}