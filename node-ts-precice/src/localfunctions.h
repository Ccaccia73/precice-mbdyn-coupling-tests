/*
 * localfunctions.h
 *
 *  Created on: Feb 19, 2018
 *      Author: claudio
 */

#ifndef LOCALFUNCTIONS_H_
#define LOCALFUNCTIONS_H_

#include <vector>
#include <iostream>

#include <mbcxx.h>


#define INVALID_SOCKET -1
#define TS_SOCK_PATH "./mbdyn.ts.sock"
#define NODE_SOCK_PATH "./mbdyn.node.sock"


void readFoil(std::vector<double>& x, std::vector<double>& y, const float c, const double *shift, const char* filename);

void ts_socket(int &socket_connected, double &timestep, int &sock_save);

void setDisplacements(double* displacements, double* coords, int size, double* ca, double *oldPos, MBCNodal *node);

void computeForces(double* forces, double* coords, int size, double* ca, MBCNodal *node);

#endif /* LOCALFUNCTIONS_H_ */
