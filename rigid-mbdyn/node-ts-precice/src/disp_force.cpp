
#include <math.h>
#include <iostream>
#include <fstream>
#include "localfunctions.h"


void setDisplacements(double* displacements, double* coords, int size, double* ca, double *oldPos, MBCNodal *node ){

	double dy = node->X(1,2) - oldPos[0];
	std::cout << "computing displacement:" << std::endl;
	std::cout << "Old position: " << oldPos[0] << " - Actual pos: " << node->X(1,2) << " - delta: " << dy << std::endl;
	// double dtheta = node->Theta(1,3) - oldPos[1];

	for(int i = 0; i < size; i++){
		displacements[2*i] = 0.0; //(coords[2*i+1]-ca[1])*dtheta;
		displacements[2*i+1] = dy; // +  (coords[2*i]-ca[0])*dtheta;
	}

}


void computeForces(double* forces, double* coords, int size, double* ca, MBCNodal *node){

	// double fx = 0.0;
	double fy = 0.0;
	double mz = 0.0;

	for(int i = 0; i < size; i++){
		// fx += forces[2*i];
		fy += forces[2*i+1];
		mz = mz - forces[2*i]*(coords[2*i+1] - ca[1]) + forces[2*i+1]*(coords[2*i]-ca[0]);
	}

	node->F(1,1) = 0.0;
	node->F(1,2) = fy;
	node->F(1,3) = 0.0;

	node->M(1,1) = 0.0;
	node->M(1,2) = 0.0;
	node->M(1,3) = 0.0; //mz;


}


