
#include <mbcxx.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include "localfunctions.h"


int main(void) {

	std::cout << "initializie timestep data... " << std::endl;

	int sockconn = 0;
	int socksave = -1;
	//double time = 0.0;
	double dt = 0.01;
	int count = 0;

	std::cout << "send first timestep. (needed?)" << std::endl;

	ts_socket(sockconn, dt, socksave);


	std::cout << "initialize parameters for external force." << std::endl;

	int retcode;

	MBCBase::Rot refnode = MBCBase::NONE;
	unsigned int nodes = 1;
	bool labels = false;
	MBCBase::Rot rot = MBCBase::THETA;
	bool accels = true;

	MBCNodal* node = new MBCNodal(refnode, nodes, labels, rot, accels );

	retcode = node->Init(NODE_SOCK_PATH);

	std::cout << "Init retcode: " << retcode << std::endl;

	node->SetTimeout(-1);

	std::cout << "Set timeout..." << std::endl;

	node->SetVerbose(true);

	std::cout << "Set verbose..." << std::endl;

	node->SetDataAndNext(true);

	std::cout << "Set Data&Next..." << std::endl;

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

	std::cout << "Initialize parameters... " << std::endl;

	// data
	double m = 1.0;
	double omega = 10.*2*M_PI;
	double xi = 0.1;
	double k = m*pow(omega,2);
	double d = 2*xi*m*omega;

	//int i = 0;
	//int maxiter = 5;

	std::cout << "Start" << std::endl;



	while(true){


		if(count == 0){
			dt = 0.01;
		}else if (count == 1) {
			dt = 0.02;
		}else{
			dt = 0.03;
		}

		std::cout << "dt: " << dt << std::endl;

		count = (count+1)%3;

		ts_socket(sockconn, dt, socksave);

		std::cout << "get motion" << std::endl;
		retcode = node->GetMotion();

		std::cout << "Get motion retcode: " << retcode << std::endl;

		if(retcode){
			break;
		}

		for(int kk = 1; kk<4; kk++){
			node->F(1,kk) = -k*node->X(1,kk) - d*node->XP(1,kk);
		}

		node->PutForces(true);
	};

	delete node;
	return EXIT_SUCCESS;
}
