/*
 * RigidMotion.cpp
 *
 *  Created on: Jan 31, 2018
 *      Author: claudio
 */

#include "precice/SolverInterface.hpp"
#include "boost/format.hpp"

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>

#define _USE_MATH_DEFINES
#include <math.h>

// #include <stdio.h>

void readFoil(std::vector<double>& x, std::vector<double>& y, const float c, const double *shift, const char* filename){

	std::string line;
	float data[2];

	std::ifstream infile(filename);

	if(infile.fail()){
		std::cout << "File not open" << std::endl;
		return;
	}

	// read first line which is title
	std::getline(infile, line);
	std::cout << "Reading foil data for " << line << std::endl;

	while(infile >> data[0] >> data[1]){
		x.push_back(data[0]*c+shift[0]);
		y.push_back(data[1]*c+shift[1]);
	}

	/*
	std::cout << "X coord: " << x.size() << std::endl;
	std::cout << "Y coord: " << y.size() << std::endl;

	for(std::vector<float>::iterator it = y.begin(); it != y.end(); ++it) {
	    std::cout << *it << " ";
	}
	*/
}


void setDisplacements(double* displacements, double* coords, int size, double* ca, double t, double dt, std::ofstream& file){

	const double omega_d = -1.5*2; // x2 dato che il movimento si ha solo nel primo mezzo secondo

	const double omega = omega_d*M_PI/180.0;

	double fractpart, intpart, dalpha;

	fractpart = modf(t, &intpart);

	for(int i = 0; i < size; i++){
		if(fractpart < 0.5){
			dalpha = omega_d*dt;
			displacements[2*i] =  -(coords[2*i+1]-ca[1])*omega*dt;
			displacements[2*i+1] = (coords[2*i]-ca[0])*omega*dt;
			//coords[2*i] += displacements[2*i];
			//coords[2*i+1] += displacements[2*i+1];
		}else{
			dalpha = 0.0;
			displacements[2*i] =  0.0; //(coords[2*i+1]-ca[1])*alpha0*delta;
			displacements[2*i+1] = 0.0;//h0*delta +  (coords[2*i]-ca[0])*alpha0*delta;
		}
	}
	// std::cout << "timestep: " << t << "\tx: " << coords[size-1] << "\ty: " << coords[size] << "\tdt: " << dt << std::endl;
	// std::cout << "timestep: " << t << "\tDx: " << displacements[size-1] << "\tDy: " << displacements[size] << std::endl;

	file << t << "\t" << dt << "\t" << dalpha << "\t" << coords[size-1] << "\t" << coords[size] << "\t" << displacements[size-1] << "\t" << displacements[size] << "\t";
}


void computeForces(double* forces, double* coords, int size, double* ca, double t, std::ofstream& file){
	double fx = 0.0;
	double fy = 0.0;
	double mz = 0.0;

	for (int i = 0; i < size; ++i) {
		fx += forces[2*i];
		fy += forces[2*i+1];
		mz = mz - forces[2*i]*(coords[2*i+1] - ca[1]) + forces[2*i+1]*(coords[2*i]-ca[0]);
	}
	std::cout << "timestep: " << boost::format("%1.3f") % t << "\tFx: " << boost::format("%5.2e") % fx << "\tFy: " << boost::format("%5.2e") % fy << "\tMz: " << boost::format("%5.2e") % mz << std::endl;

	file << fx << "\t" << fy << "\t" << mz << std::endl;
}


int main(int argc, char **argv) {

	using namespace precice;
	using namespace precice::constants;

	std::vector<double> xCoord;
	std::vector<double> yCoord;


	double chord = 1.0;

	double ca[2] = {0.0, 0.0};
	double offset[2] = {-chord/4. , 0.0};
	//double h0 = 10.0;
	//double alpha0 = -5.0*(M_PI/180.0);
	//double omega0 = M_PI/1.25;

	std::string configFileName(argv[1]);

	std::cout << "Read foil coordinates..." << std::endl;

	readFoil(xCoord, yCoord, chord, offset, argv[2]);

	std::cout << "X coord: " << xCoord.size() << std::endl;
	std::cout << "Y coord: " << yCoord.size() << std::endl;

	std::cout << std::endl;

	std::cout << "Starting Structure Solver..." << std::endl;

	std::string dummyName = "Rigid";


	/* interface parameters:
		- name:
		- process index
		- process size

	*/
	SolverInterface interface(dummyName, 0, 1);
	interface.configure(configFileName);
	std::cout << "preCICE configured..." << std::endl;

	const std::string& coric = precice::constants::actionReadIterationCheckpoint();
	const std::string& cowic = precice::constants::actionWriteIterationCheckpoint();

	int dim = interface.getDimensions();

	int meshID = interface.getMeshID("Rigid_Mesh");

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

	int displID = interface.getDataID("DisplacementDeltas0", meshID);
	int forceID = interface.getDataID("Forces0", meshID);

	double forces[vertexSize*dim];
	double displacements[vertexSize*dim];


	double dt = 0.01; // solver timestep size
	double precice_dt; // maximum precice timestep size

	double t = 0.0;

	precice_dt = interface.initialize();


	// open file to write forces
	std::ofstream outputFile("Forces_Displacements.txt");

	outputFile << "t\tdt\tda\tx\ty\tdx\tdy\tFx\tFy\tMz" << std::endl;

	std::cout << "Start" << std::endl;

	if (interface.isReadDataAvailable()){
		interface.readBlockVectorData(forceID, vertexSize, vertexIDs, forces);
	}


	while (interface.isCouplingOngoing()){
	  if(interface.isActionRequired(cowic)){
	    // saveOldState(); // save checkpoint
	    interface.fulfilledAction(cowic);
	  }

	  // beginTimeStep(); // e.g. compute adaptive dt


	  dt = std::min(precice_dt, dt);

	  setDisplacements(displacements, coords, vertexSize, ca, t, dt, outputFile);

	  interface.writeBlockVectorData(displID, vertexSize, vertexIDs, displacements);

	  precice_dt = interface.advance(dt);

	  interface.readBlockVectorData(forceID, vertexSize, vertexIDs, forces);


	  if(interface.isActionRequired(cowic)){
		  interface.fulfilledAction(cowic);
	  }

	  if(interface.isActionRequired(coric)){ // timestep not converged
	    // reloadOldState(); // set variables back to checkpoint
	    interface.fulfilledAction(coric);
	  }
	  else{ // timestep converged
		  // computeTimeStep();
		  t += dt;
		  // endTimeStep(); // e.g. update variables, increment time
		  for(int i = 0; i < vertexSize; i++){
			  coords[2*i] += displacements[2*i];
			  coords[2*i+1] += displacements[2*i+1];
		  }

		  computeForces(forces, coords, vertexSize, ca, t, outputFile);

	  }
	}

	outputFile.close();

	interface.finalize();

	return 0;

}


