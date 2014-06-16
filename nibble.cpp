/*
 * File:   nibble.cpp
 * Author: sondra
 *
 * Created on June 15, 2014, 12:38 PM
 */

#include <common/mavlink.h>

#include "Comm.h"


int main(int argc, char **argv) {

    Mission mission;
    Comm comm;
    comm.Startup(argc, argv);

    while (true) {
        comm.ReadMessages(&mission);
        mission.HandleMission(&comm);
    }


}


