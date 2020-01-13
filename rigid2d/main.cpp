#include "rigid2d.hpp"
#include <iostream>

int main(){
	rigid2d::Transform2D Tab;
	rigid2d::Transform2D Tbc;
	rigid2d::Transform2D Tba;
	rigid2d::Transform2D Tcb;
	rigid2d::Transform2D Tac;
	rigid2d::Transform2D Tca;

	// rigid2d::Vector2D vec;
	std::cout << "When prompted enter two transforms: Tab and Tbc\n";
	std::cout << "Please enter theta, x, & y of Tab\n";
	std::cin >> Tab;
	std::cout << "Thank you, now please enter theta, x, & y of Tbc\n";
	std::cin >> Tbc;
	
	Tba = Tab.inv();
	Tcb = Tbc.inv();

	std::cout << "Tab:\n"<< Tab;
	std::cout << "Tbc:\n"<< Tbc;
	std::cout << "Tba:\n"<< Tba;
	std::cout << "Tcb:\n"<< Tcb;

	Tac = Tab * Tbc;
	Tca = Tac.inv(); 
	std::cout << "Tac:\n"<< Tac;
	std::cout << "Tca:\n"<< Tca;

	char frame;
	rigid2d::Vector2D v;
	rigid2d::Vector2D va;
	rigid2d::Vector2D vb;
	rigid2d::Vector2D vc;


	std::cout << "Awesome! Now enter v (specifying the x and y values) & a refrence frame (i.e. a, b, orc)\n";
	std::cin >> v;
	std::cin >> frame;
	
	switch (frame){
		case 'a':
			va = v;
			vb = Tba(v);
			vc = Tca(v);
			break;
		case 'b':
			vb = v;
			va = Tab(v);
			vc = Tcb(v);
			break;
		case 'c':
			vc = v;
			va = Tac(v);
			vb = Tbc(v);
			break;
		default:
			std::cout << "Ya goofed my dude\n";
	}
	std::cout << "Va:\n";
	std::cout << va;
	std::cout << "Vb:\n";
	std::cout << vb;
	std::cout << "Vc:\n";
	std::cout << vc;


}