/*
 * localfunctions.h
 *
 *  Created on: Feb 19, 2018
 *      Author: claudio
 */

#ifndef LOCALFUNCTIONS_H_
#define LOCALFUNCTIONS_H_


#define INVALID_SOCKET -1
#define TS_SOCK_PATH "../mbdyn.ts.sock"
#define NODE_SOCK_PATH "../mbdyn.node.sock"

void ts_socket(int &socket_connected, double &timestep, int &sock_save);



#endif /* LOCALFUNCTIONS_H_ */
