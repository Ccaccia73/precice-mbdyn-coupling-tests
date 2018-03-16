
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "localfunctions.h"



void ts_socket(int &socket_connected, double &timestep, int &sock_save){


    int dble_size, sun_size;
    // int sock;
    struct sockaddr_un sun_remote;

    dble_size = sizeof(double);
    // sock = *sock_save;

    if (!socket_connected) {

        memset(&sun_remote, 0, sizeof(sun_remote));
        sun_size = sizeof(sun_remote);

        if( (sock_save = socket(AF_UNIX, SOCK_STREAM, 0) ) == INVALID_SOCKET) {
            perror("socket()");
            exit(errno);
        }

        sun_remote.sun_family = AF_UNIX;
        strcpy(sun_remote.sun_path, TS_SOCK_PATH);

        if (connect(sock_save, (struct sockaddr *) &sun_remote, sun_size) == -1) {
            perror("connect()");
            exit(errno);
        }
        else {
            socket_connected = 1;
        }

    }

    if (send(sock_save, &timestep, dble_size, 0) == -1) {
        perror("send()");
        exit(errno);
    }

};

