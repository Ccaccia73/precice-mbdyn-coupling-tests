
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


	/************* TIME SOCKET *****************/

	// timestep socket configuration
	std::cout << "initializie timestep data... " << std::endl;


	// start time and computation step
	// TODO: timesteps are written in many places (SU2 config, PreCICE config, MBDyn, here... )
	double t = 0.0;
	double dt = 0.005;


	int sockconn = 0;
	int socksave = -1;
	std::cout << "send first timestep. (needed)" << std::endl;
	ts_socket(sockconn, dt, socksave);


	std::cout << "initialize parameters for external force." << std::endl;


	/************ EXTERNAL NODE ****************/

	// EXTERNAL NODE configuration (single node)

	// return code of MBDyn external node calls
	int retcode;

	// Ext Node parameters (as seen in the example)
	MBCBase::Rot refnode = MBCBase::NONE;
	unsigned int nodes = 1;
	bool labels = false;
	MBCBase::Rot rot = MBCBase::THETA;
	bool accels = true;

	// create new node
	MBCNodal* node = new MBCNodal(refnode, nodes, labels, rot, accels );

	// Initialize socket: sequence of calls as seen in the example
	retcode = node->Init(NODE_SOCK_PATH);
	std::cout << "Init retcode: " << retcode << std::endl;

	node->SetTimeout(-1);
	std::cout << "Set timeout..." << std::endl;

	node->SetVerbose(true);
	std::cout << "Set verbose..." << std::endl;

	node->SetDataAndNext(true);
	std::cout << "Set Data&Next..." << std::endl;

	retcode = node->Negotiate();
	std::cout << "Negotiate retcode: " << retcode << std::endl;

	// initialize force and moment (should not be needed)
	/*
	node->F(1,1) = 0.0;
	node->F(1,2) = 0.0;
	node->F(1,3) = 0.0;

	node->M(1,1) = 0.0;
	node->M(1,2) = 0.0;
	node->M(1,3) = 0.0;
	*/


	/*********  PreCICE configuration  **********/
	std::cout << "Initialize Precice parameters... " << std::endl;

	using namespace precice;
	using namespace precice::constants;

	// vectors containing coordinates of airfoil mesh points
	std::vector<double> xCoord;
	std::vector<double> yCoord;

	// chord length and offset:
	// file with mesh point has tip at (0,0) and lenght=1
	double chord = 1.0;
	double offset[2] = {-chord/4. , 0.0};

	// position of MBDyn node
	double ca[2] = {0.0, 0.0};

	// old position: needed to compute displacement deltas:
	double oldPos[2] = {0.0, 0.0};

	// name of the PreCICE config name (first argument)
	std::string configFileName(argv[1]);

	// names of participants, mesh, parameters, etc... (same as PreCICE config file)
	std::string meshName = "Rigid_Mesh";
	std::string readDataName = "DisplacementDeltas0";
	std::string writeDataName = "Forces0";
	std::string participantName = "MBDyn";


	// Read airfoil coordinates: = "structural mesh"
	std::cout << "Read foil coordinates..." << std::endl;
	readFoil(xCoord, yCoord, chord, offset, argv[2]);

	// print number of points:
	std::cout << "X coord: " << xCoord.size();
	std::cout << std::endl;

	std::cout << "Y coord: " << yCoord.size();
	std::cout << std::endl;

	std::cout << std::endl;

	std::cout << "Starting Structure Solver..." << std::endl;


	// Configure PreCICE interface
	/* interface parameters:
		- name:
		- process index
		- process size

	*/
	SolverInterface interface(participantName, 0, 1);
	interface.configure(configFileName);
	std::cout << "preCICE configured..." << std::endl;

	// define strings used for write and read checkpoints
	const std::string& coric = actionReadIterationCheckpoint();
	const std::string& cowic = actionWriteIterationCheckpoint();

	// get problem dimension: 2D here
	int dim = interface.getDimensions();


	// Build array with structural mesh coordinates:
	//
	// [x0, y0, x1, y1, .... ,x(n-1), y(n-1)]
	// the array is updated each completed step with current coordinate positions
	//
	int meshID = interface.getMeshID(meshName);

	int vertexSize = xCoord.size(); // number of vertices at wet surface

	// determine vertexSize
	double coords[vertexSize*dim];
	int vertexIDs[vertexSize];

	// coords of vertices at wet surface, initialized with airfoil data
	for(int i = 0; i < vertexSize; i++){
		coords[2*i] = xCoord[i];
		coords[2*i+1] = yCoord[i];
		vertexIDs[i] = i;
	}

	// set interface vertices
	interface.setMeshVertices(meshID, vertexSize, coords, vertexIDs);

	// build vector containing node forces and displacements:
	//
	// [Fx0, Fy0, Fx1, Fy1, ... Fx(n-1), Fy(n-1)]
	// [dx0, dy0, dx1, dy1, ... dx(n-1), dy(n-1)]
	int displID = interface.getDataID(readDataName, meshID);
	int forceID = interface.getDataID(writeDataName, meshID);

	double forces[vertexSize*dim] = {0.0};
	double displacements[vertexSize*dim] = {0.0};

	// solver timestep size
	// maximum PreCICE timestep size
	// TODO: how does it relate to all the timesteps?
	double precice_dt = interface.initialize();


	// Create a file to write simulation data at each timestep (used for debug/analysis purposes)
	// open file to write forces
	std::ofstream outputFile("Forces_Displacements.txt");

	outputFile << "#t\tdt\toldY\t\toldTh\txTail\tyTail\txTip\tyTip\tdxTail\tdyTail\tdxTip\tdyTip\tFy" << std::endl;

	// begin simulation
	std::cout << "Start" << std::endl;


	// skeleton of the coupling retrieved from:
	// - 1delastictube (github repository example)
	// - adapter example: https://github.com/precice/precice/wiki/Adapter-Example


	// Write initial data if required
	if (interface.isActionRequired(actionWriteInitialData())) {
		interface.writeBlockVectorData(forceID, vertexSize, vertexIDs, forces);
		interface.fulfilledAction(actionWriteInitialData());
	}

	// initialize interface data
	interface.initializeData();

	// Read initial data if available
	if (interface.isReadDataAvailable()){
		interface.readBlockVectorData(displID, vertexSize, vertexIDs, displacements);
	}


	// Start coupling:
	while (interface.isCouplingOngoing()){

		if(interface.isActionRequired(cowic)){
			// When an implicit coupling scheme is used, checkpointing is required
			std::cout << "--- cowic ---" << std::endl;
			std::cout << cowic << std::endl;
			// stub of function as described in the adapter example
			// saveOldState(); // save checkpoint
			interface.fulfilledAction(cowic);
		}


		// Send dt to MBDyn
		std::cout << "send dt (" << precice_dt <<  ") to MBDyn... " << std::endl;
		ts_socket(sockconn, precice_dt, socksave);


		// MBDyn compute motion step
		std::cout << "compute motion step" << std::endl;

		retcode = node->GetMotion();

		std::cout << "Get motion retcode: " << retcode << std::endl;

		if(retcode){
			break;
		}

		// compute mesh motion
		setDisplacements(displacements, coords, vertexSize, ca, oldPos, node);
		// write data to interface
		interface.writeBlockVectorData(displID, vertexSize, vertexIDs, displacements);

		// advance
		precice_dt = interface.advance(precice_dt);

		// read forces from interface
		interface.readBlockVectorData(forceID, vertexSize, vertexIDs, forces);
		// compute force and moment at MDbyn node
		computeForces(forces, coords, vertexSize, ca, node);


		// Checkpoint
		/*
		if(interface.isActionRequired(cowic)){
			// When an implicit coupling scheme is used, checkpointing is required
			// stub of function as described in the adapter example
			// saveOldState(); // save checkpoint
			std::cout << "--- cowic 2 ---" << std::endl;
			std::cout << cowic << std::endl;
			interface.fulfilledAction(cowic);
		}
		*/


		if(interface.isActionRequired(coric)){
			// timestep not converged
			// reloadOldState(); // set variables back to checkpoint
			node->PutForces(false);
			interface.fulfilledAction(coric);
			std::cout << "---- NOT Converged ----" << std::endl;
		}else{
			// timestep converged
			node->PutForces(true);
			std::cout << "**** Converged ****" << std::endl;
			t += precice_dt;
			oldPos[0] = node->X(1,2);
			oldPos[1] = node->Theta(1,3);

			// update mesh nodes coordinates
			for(int i = 0; i < vertexSize; i++){
				coords[2*i] += displacements[2*i];
				coords[2*i+1] += displacements[2*i+1];
			}

			// write data to file
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

	outputFile.close();
	delete node;
	return EXIT_SUCCESS;
}

