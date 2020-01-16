#include "rigid2d/rigid2d.hpp"
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
	rigid2d::Twist2D V;
	rigid2d::Twist2D Va;
	rigid2d::Twist2D Vb;
	rigid2d::Twist2D Vc;

	std::cout << "Awesome! Now enter a vector (specifying the x and y values) & a refrence frame (i.e. a, b, orc)\n";
	std::cin >> v;
	std::cin >> frame;
	std::cout << "Enter a twist (omega, vx, vy) in the same frame as the previous vector: \n";
	std::cin >> V;
	
	switch (frame){
		case 'a':
			//cache vector and twist in frame a
			va = v;
			Va = V;
			//convert vector and twist to frames b and c
			vb = Tba(va);
			vc = Tca(va);
			Vb = Tba.adjoint(Va);
			Vc = Tca.adjoint(Va);
			break;
		case 'b':
			//cache vector and twist in frame b
			vb = v;
			Vb = V;
			//convert vector and twist to frames a and c
			va = Tab(vb);
			vc = Tcb(vb);
			Va = Tab.adjoint(Vb);
			Vc = Tcb.adjoint(Vb);
			break;
		case 'c':
			//cache vector and twist in frame c
			vc = v;
			Vc = V;
			//convert vector and twist to frames a and b
			va = Tac(vc);
			vb = Tbc(vc);
			Va = Tac.adjoint(Vc);
			Vb = Tbc.adjoint(Vc);
			break;
		default:
			std::cout << "Ya goofed my dude\n";
	}

	/*Output*/
	std::cout << "Vectors in frames a,b,c:\n";
	std::cout << "v_a:\t";
	std::cout << va;
	std::cout << "v_b:\t";
	std::cout << vb;
	std::cout << "v_c:\t";
	std::cout << vc;
	std::cout << "Twists in frames a,b,c:\n";
	std::cout << "Va:\t";
	std::cout << Va;
	std::cout << "Vb:\t";
	std::cout << Vb;
	std::cout << "Vc:\t";
	std::cout << Vc;
}