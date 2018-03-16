#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <math.h>


#define INVALID_SOCKET -1
#define TS_SOCK_PATH "./mbdyn.ts.sock"


void ts_socket(int &socket_connected, double &timestep, int &sock_save){


    int dble_size, sun_size;
    struct sockaddr_un sun_remote;

    dble_size = sizeof(double);

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

    //*sock_save = sock;

    if (send(sock_save, &timestep, dble_size, 0) == -1) {
        perror("send()");
        exit(errno);
    }

};



int main(int argc, char **argv) {

	double t, ti, tf, a, b, c, k, dti;

    double dt;

    ti = 0.0;
    tf = 1.0;

    int sk;
	int sk_conn;

    sk = 0;
    sk_conn = -1;

    k = 1.1;
    dti = 1.e-3;
    t = ti + dti;
    a = (4.*dti*(k-1.))/(3.*pow(ti,2)+2.*ti*tf-pow(tf,2));
    b = -(4.*dti*(k-1.))/(3.*ti-tf);
    c = dti+(4.*dti*(k-1.)*ti*tf)/(3.*pow(ti,2)+2.*ti*tf-pow(tf,2));

    std::cout << "t: " << t << std::endl;

    while (t < tf){
         // an arbitrary law for the time step...
         dt = a*pow(t,2) + b*t + c;

         std::cout << "dt: " << dt << std::endl;

         ts_socket(sk, dt, sk_conn);

         t = t + dt;

         std::cout << "t: " << t << std::endl;
    }


}


