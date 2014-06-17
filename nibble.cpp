/*
 * File:   nibble.cpp
 * Author: sondra
 *
 * Created on June 15, 2014, 12:38 PM
 */

#include <common/mavlink.h>
#include <pthread.h>

#include "Comm.h"
#include "ComputerVision.h"


int main(int argc, char **argv) {

    Mission mission;
    Comm comm;
    comm.Startup(argc, argv);

    pthread_t cvThread;
    int rc;

    printf( "Starting Computer Vision pthread.");
    rc = pthread_create(&cvThread, NULL, ComputerVision::RunCV, NULL);
    if (rc) {
        printf("**************Error starting CV thread, return code from pthread_create is %d****************\n", rc);
    }

    while (true) {
        comm.ReadMessages(&mission);
        mission.HandleMission(&comm);
    }


}


