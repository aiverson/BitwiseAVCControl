/*
 * File:   nibble.cpp
 * Author: sondra
 *
 * Created on June 15, 2014, 12:38 PM
 */

#include <common/mavlink.h>
#include <pthread.h>
#include <stdio.h>

#include "BalloonLocation.h"
#include "Comm.h"
#include "ComputerVision.h"

pthread_mutex_t locationLock = PTHREAD_MUTEX_INITIALIZER;
balloonLocation_t location;

int main(int argc, char **argv) {

    Mission mission;
    Comm comm;
    comm.Startup(argc, argv);

    pthread_t cvThread;
    int rc;

    printf( "Starting Computer Vision pthread.");
    //pthread_mutex_init(&locationLock, NULL);
    location.range = -1.0;
    location.phi = 0.0;
    location.theta = 0.0;
    gettimeofday(&location.timestamp, NULL);

    rc = pthread_create(&cvThread, NULL, ComputerVision::RunCV, NULL);
    if (rc) {
        printf("**************Error starting CV thread, return code from pthread_create is %d****************\n", rc);
    }

    while (true) {
        comm.ReadMessages(&mission);
        mission.HandleMission(&comm);
    }


}


