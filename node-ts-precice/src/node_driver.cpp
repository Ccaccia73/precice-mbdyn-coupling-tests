
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>


#include <stdio.h>
#include <stdlib.h>



#include <mbcxx.h>
#include <precice/SolverInterface.hpp>

#include "localfunctions.h"


int main(int argc, char **argv) {


	// timestep socket configuration
	std::cout << "initializie timestep data... " << std::endl;

	int sockconn = 0;
	int socksave = -1;
	double time = 0.0;
	double dt = 0.005;
	std::cout << "send first timestep. (needed)" << std::endl;
	ts_socket(sockconn, dt, socksave);


	std::cout << "initialize parameters for external force." << std::endl;


	// EXTERNAL NODE configuration
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

	// initialize force and moment
	/*
	node->F(1,1) = 0.0;
	node->F(1,2) = 0.0;
	node->F(1,3) = 0.0;

	node->M(1,1) = 0.0;
	node->M(1,2) = 0.0;
	node->M(1,3) = 0.0;
	*/


	// PreCICE configuration
	std::cout << "Initialize Precice parameters... " << std::endl;


	using namespace precice;
	using namespace precice::constants;

	std::vector<double> xCoord;
	std::vector<double> yCoord;


	double chord = 1.0;

	double ca[2] = {0.0, 0.0};
	double offset[2] = {-chord/4. , 0.0};
	double oldPos[2] = {0.0, 0.0};

	std::string configFileName(argv[1]);
	std::string meshName = "Rigid_Mesh";
	std::string readDataName = "DisplacementDeltas0";
	std::string writeDataName = "Forces0";
	std::string participantName = "MBDyn";


	std::cout << "Read foil coordinates..." << std::endl;

	readFoil(xCoord, yCoord, chord, offset, argv[2]);

	std::cout << "X coord: " << xCoord.size() << std::endl;
	std::cout << "Y coord: " << yCoord.size() << std::endl;

	std::cout << std::endl;

	std::cout << "Starting Structure Solver..." << std::endl;



	/* interface parameters:
		- name:
		- process index
		- process size

	*/
	SolverInterface interface(participantName, 0, 1);
	interface.configure(configFileName);
	std::cout << "preCICE configured..." << std::endl;

	const std::string& coric = actionReadIterationCheckpoint();
	const std::string& cowic = actionWriteIterationCheckpoint();

	int dim = interface.getDimensions();

	int meshID = interface.getMeshID(meshName);

	int vertexSize = xCoord.size(); // number of vertices at wet surface

	// determine vertexSize
	double coords[vertexSize*dim];
	int vertexIDs[vertexSize];

	// coords of vertices at wet surface
	for(int i = 0; i < vertexSize; i++){
		coords[2*i] = xCoord[i];
		coords[2*i+1] = yCoord[i];
		vertexIDs[i] = i;
	}

	interface.setMeshVertices(meshID, vertexSize, coords, vertexIDs);

	int displID = interface.getDataID(readDataName, meshID);
	int forceID = interface.getDataID(writeDataName, meshID);

	double forces[vertexSize*dim];
	double displacements[vertexSize*dim];


	// double dt = 0.01; // solver timestep size
	double precice_dt = interface.initialize();; // maximum precice timestep size

	double t = 0.0;


	// open file to write forces
	std::ofstream outputFile("Forces_Displacements.txt");

	outputFile << "#t\tdt\toldY\t\toldTh\txTail\tyTail\txTip\tyTip\tdxTail\tdyTail\tdxTip\tdyTip\tFy" << std::endl;


	std::cout << "Start" << std::endl;

	if (interface.isReadDataAvailable()){
		interface.readBlockVectorData(forceID, vertexSize, vertexIDs, forces);
	}

	while (interface.isCouplingOngoing()){
		if(interface.isActionRequired(cowic)){
			std::cout << "--- cowic ---" << std::endl;
			std::cout << cowic << std::endl;
			// saveOldState(); // save checkpoint
			interface.fulfilledAction(cowic);
		}

		// beginTimeStep(); // e.g. compute adaptive dt
		// dt = std::min(precice_dt, dt);

		std::cout << "send dt (" << precice_dt <<  ") to MBDyn... " << std::endl;
		// ts_socket(sockconn, dt, socksave);
		ts_socket(sockconn, precice_dt, socksave);

		std::cout << "compute motion step" << std::endl;

		retcode = node->GetMotion();

		std::cout << "Get motion retcode: " << retcode << std::endl;

		if(retcode){
			break;
		}

		// compute motion
		setDisplacements(displacements, coords, vertexSize, ca, oldPos, node);
		interface.writeBlockVectorData(displID, vertexSize, vertexIDs, displacements);

		// advance
		precice_dt = interface.advance(precice_dt);

		// read forces
		interface.readBlockVectorData(forceID, vertexSize, vertexIDs, forces);
		computeForces(forces, coords, vertexSize, ca, node);

		if(interface.isActionRequired(cowic)){
			std::cout << "--- cowic 2 ---" << std::endl;
			std::cout << cowic << std::endl;
			interface.fulfilledAction(cowic);

		}

		if(interface.isActionRequired(coric)){ // timestep not converged
		  // reloadOldState(); // set variables back to checkpoint
		  node->PutForces(false);
		  interface.fulfilledAction(coric);
		  std::cout << "---- NOT Converged ----" << std::endl;
		}
		else{ // timestep converged
			node->PutForces(true);
			std::cout << "**** Converged ****" << std::endl;
			t += precice_dt;
			oldPos[0] = node->X(1,2);
			oldPos[1] = node->Theta(1,3);

			for(int i = 0; i < vertexSize; i++){
				coords[2*i] += displacements[2*i];
				coords[2*i+1] += displacements[2*i+1];
			}

			std::cout << "tstep: " << t << "\tdt: " << precice_dt << "\ty: " << coords[vertexSize] << "\tdy: " << displacements[vertexSize]  << std::endl;
			std::cout  << "\tFy: " << node->F(1,2) << std::endl;

			outputFile << t << "\t" << precice_dt << "\t" << oldPos[0] << "\t" << oldPos[1];
			outputFile<< "\t" << coords[0] << "\t" << coords[1];
			outputFile<< "\t" << coords[vertexSize-1] << "\t" << coords[vertexSize];
			outputFile << "\t" << displacements[0] << "\t" << displacements[1];
			outputFile << "\t" << displacements[vertexSize-1] << "\t" << displacements[vertexSize];
			outputFile << "\t" << node->F(1,2) << std::endl;

		}

	};

	interface.finalize();

	delete node;
	return EXIT_SUCCESS;
}

