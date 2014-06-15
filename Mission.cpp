/* 
 * File:   Mission.cpp
 * Author: sondra
 * 
 * Created on June 15, 2014, 11:05 AM
 */

#include "Mission.h"
#include <stdio.h>

Mission::Mission() {
    originalMissionItemCount = 0;
    reportedMissionItemCount = 0;

}

Mission::Mission(const Mission& orig) {
}

Mission::~Mission() {
}


bool Mission::StoreMissionItem( mavlink_mission_item_t item) {
    printf ("Storing mission item %d", item.seq);
    originalMission[item.seq] = item;
    originalMissionItemCount++;

    printf( "Total mission items is now %d", originalMissionItemCount );

    return true;
}



