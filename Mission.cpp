/* 
 * File:   Mission.cpp
 * Author: sondra
 * 
 * Created on June 15, 2014, 11:05 AM
 */

#include "Mission.h"
#include <stdio.h>

Mission::Mission() {
    currState = BOOTING;
    receivedMissionItemCount = 0;
    missionItemCount = -1;
    loopCounter = 0;
}

Mission::Mission(const Mission& orig) {
}

Mission::~Mission() {
}


int Mission::GetMissionItemCount() {
    return missionItemCount;
}

int Mission::GetReceivedMissionItemCount() {
    return receivedMissionItemCount;
}

void Mission::SetMissionCount( int missionCount ) {

    missionItemCount = missionCount;
}


bool Mission::StoreMissionItem( mavlink_mission_item_t item) {
    printf ("+++++++++++++++++++++In Mission:  Storing mission item %d\n", item.seq);
    mission[item.seq] = item;

    // Make sure it is the mission item we wanted.  Not a duplicate.
    if (item.seq == receivedMissionItemCount)
        receivedMissionItemCount++;

    printf( "+++++++++++++++++++++In Mission:  Total mission items received so far is %d out of %d\n", receivedMissionItemCount, missionItemCount );

    if (receivedMissionItemCount == missionItemCount)
        PrintMission();

    return true;
}

void Mission::PrintMission() {
    int i;

    printf("\n+++++++++++++++++++++++++++++++++++++++++++\n");
    printf("\t all mission items:\tseq \tcommand \tparam1 \tparam2 \tparam3 \tparam4 \tx \ty \tz \ttgt_sys\ttgt_cmp\tframe \tcurrent \tautocontinue\n");
    for (i=0; i<missionItemCount; i++) {
        printf("\t mission item %d: \t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%d\n",
                i,
                mission[i].seq,
                mission[i].command,
                mission[i].param1,
                mission[i].param2,
                mission[i].param3,
                mission[i].param4,
                mission[i].x,
                mission[i].y,
                mission[i].z,
                mission[i].target_system,
                mission[i].target_component,
                mission[i].frame,
                mission[i].current,
                mission[i].autocontinue);
    }
    printf("\n+++++++++++++++++++++++++++++++++++++++++++\n");

}

void Mission::HandleMission(Comm *comm) {
  
//    struct timeval tv;
//    gettimeofday(&tv, NULL);

    switch (currState) {
        case BOOTING:
            // Give the pixhawk time to startup.
            if (loopCounter > 3000) {
                printf("-------------------------Switching from state BOOTING to INITIALIZE\n" );
                currState = INITIALIZE;
            }
            break;

        case INITIALIZE:
            // Get the Mission from the pixhawk.
            if (loopCounter % 1000 == 0) {
                printf("-------------------------In INITIALIZE, missionItemCount = %d, receivedMissionItemCount = %d\n",
                        missionItemCount, receivedMissionItemCount);

                // In response to the MAVLink message MISSION_REQUEST_LIST, the pixhawk only returns the count of
                // mission items.  It doesn't actually return the list.  Each item must be requested separately.
                if (missionItemCount == -1)
                    comm->SendMissionRequestList();

                // Request the mission items one at a time.  Repeating requests, if necessary.
                else if (receivedMissionItemCount < missionItemCount)
                    comm->SendMissionRequest(receivedMissionItemCount);

                else if (receivedMissionItemCount >= missionItemCount) {
                    currState = AUTO;  // We have the whole list.

                    printf("-------------------------Switching from state INITIALIZE to AUTO\n" );
                }
            }

            break;

        case AUTO:
            break;

        case GUIDED:
            break;

    }
    loopCounter++;


}
