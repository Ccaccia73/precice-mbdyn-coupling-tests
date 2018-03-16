

#include "mbcxx.h"

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

int main(void) {

	int retcode;

	MBCBase::Rot refnode = MBCBase::NONE;
	unsigned int nodes = 1;
	bool labels = false;
	MBCBase::Rot rot = MBCBase::THETA;
	bool accels = true;

	MBCNodal* node = new MBCNodal(refnode, nodes, labels, rot, accels );

	retcode = node->Init("../mbdyn.sock");

	std::cout << "Init retcode: " << retcode << std::endl;

	node->SetTimeout(-1);
	node->SetVerbose(true);

	node->SetDataAndNext(true);

	node->Negotiate();

	std::cout << "Negotiate retcode: " << retcode << std::endl;

	/* initialize force and moment
	node->F(1) = 0.0;
	node->F(2) = 0.0;
	node->F(3) = 0.0;

	node->M(1) = 0.0;
	node->M(2) = 0.0;
	node->M(3) = 0.0;
	*/

	std::cout << "Initialize data... " << std::endl;

	// data
	/*
	double m = 1.0;
	double omega = 10.*2*M_PI;
	double xi = 0.1;
	double k = m*pow(omega,2);
	double d = 2*xi*m*omega;

	int i = 0;
	int maxiter = 5;
	*/

	const double omegaF = 2.0*2*M_PI;
	double t = 0.0;
	const double dt = 0.005;
	const double F0 = 10.0;

	double tmpF;
	// open file to write forces

	std::ofstream outputFile("Forces.txt");

	outputFile << "t\tdt\tFx\tFy\tFz" << std::endl;

	std::cout << "Start" << std::endl;

	while(true){

		// std::cout << "get motion" << std::endl;
		retcode = node->GetMotion();

		// std::cout << "Get motion retcode: " << retcode << std::endl;

		if(retcode){
			break;
		}

		outputFile << t << "\t" << dt;

		// std::cout << "Compute forces" << std::endl;

		for(int kk=1;kk<=3;kk++){
			if(kk == 2){
				tmpF = F0*sin(omegaF*t);
			}else{
				tmpF = 0.0;
			}

			// std::cout << "comp: " << kk << "\tForce: " << tmpF << std::endl;

			node->F(1,kk) = tmpF;
			outputFile << "\t" << tmpF;

			// std::cout << "set component" << std::endl;

		}

		outputFile << std::endl;

		std::cout << "Send forces" << std::endl;

		node->PutForces(true);

		std::cout << "Update time " << std::endl;

		t += dt;
	};

	delete node;

	// outputFile.close();

	return EXIT_SUCCESS;
}
