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
    currMissionIndex = 0;
}

Mission::Mission(const Mission& orig) {
}

Mission::~Mission() {
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

    PrintGlobalPosition();

}

void Mission::StoreGlobalPosition( mavlink_global_position_int_t pos ) {
    globalPosition = pos;
}

void Mission::PrintGlobalPosition() {
        printf("Most recent GLOBAL_POSITION\n");
        printf("\t time_boot_ms: %d\n", globalPosition.time_boot_ms);
        printf("\t lat: %d, %f\n", globalPosition.lat, ((double)globalPosition.lat)/1e7 );
        printf("\t lon: %d, %f\n", globalPosition.lon, ((double)globalPosition.lon)/1e7 );
        printf("\t alt: %d, %f\n", globalPosition.alt, ((double)globalPosition.alt)/1000 );
        printf("\t rel alt: %d, %f\n", globalPosition.relative_alt, ((double)globalPosition.relative_alt)/1000 );
        printf("\t vx: %d, %f\n", globalPosition.vx, ((double)globalPosition.vx)/100 );
        printf("\t vy: %d, %f\n", globalPosition.vy, ((double)globalPosition.vy)/100 );
        printf("\t vz: %d, %f\n", globalPosition.vz, ((double)globalPosition.vz)/100 );
        printf("\t hdg: %d, %f\n", globalPosition.hdg, ((double)globalPosition.hdg)/100 );
        printf("\n");    
}

void Mission::StoreCurrentMissionIndex( int index ) {
    printf("-------------------------Current mission index = %d\n", index );
    currMissionIndex = index;
}

void Mission::StoreCurrentMode( FlightMode mode ) {
    if (currFlightMode != mode) {
        printf("-------------------------Current mode changed from %d to %d\n", currFlightMode, mode );
        currFlightMode = mode;
    }
}

void Mission::HandleMission(Comm *comm) {
  

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
                    currState = PREPROGRAMMED_MISSION;  // We have the whole list.

                    printf("-------------------------Switching from state INITIALIZE to PREPROGRAMMED_MISSION\n" );
                }
            }

            break;

        case PREPROGRAMMED_MISSION:

            // We are in a preprogrammed part of the mission.  Watch for our special
            // indicators that a balloon should be near.

            // May want to insert LOITER commands (or some other flag) to indicate we should
            // start searching for balloons.  

            // Store time that we started searching for a balloon.  Switch to mode SEARCHING_FOR_BALOON.


            // Only for testing purposes...
            if (loopCounter % 1000 == 0) {
                TestBalloonMutex();


/*
                if (currFlightMode != AUTO) {
                    // if not in mode AUTO, switch to it for testing...
                    printf("-------------------------Requesting mode change to AUTO.\n" );
                    comm->SendSetMode(int (AUTO) );
                    comm->SendMissionSetCurrent(1);
                }
*/
            }
            break;

        case SEARCHING_FOR_BALLOON:
            // We are still in auto mode flying a preprogrammed mission, but we are
            // also expecting to find a balloon in this segment of the mission.

            // If we find a reasonably close balloon, switch to guided mode and go pop it.
            // If find a balloon target, switch to guided flight mode and CHASING_BALLOON state.

            // If we do not find a balloon in a reasonable amount of time, switch
            // back to PREPROGRAMMED_MISSION mode.

            if (loopCounter % 5000 == 0) {
                if (currFlightMode == AUTO) {

                    if (IsBalloonNearby()) {

                        printf("-------------------------Requesting mode change to GUIDED.\n" );
                        comm->SendSetMode(int (GUIDED));
                        missionIndexWhenReturnToAuto = currMissionIndex;
                        gettimeofday(&startTime, NULL);

                        currState = CHASING_BALLOON;
                    }
                }
            }

            break;
        case CHASING_BALLOON:

            // FINISH...

            // Periodically compare newly calculated balloon location with last balloon location.
            // If altitude has changed, send waypoint with current = 3.  Changes altitude only
            // If balloon location changed, send waypoint with current = 2.  (i.e., a guided waypoint)


            // If balloon disappears (hopefully popped) or time expires, resume the preprogrammed mission,
            // (i.e., flight mode AUTO and state PREPROGRAMMED_MISSION).  The pixhawk



            if (loopCounter % 2000 == 0) {
                    printf("-------------------------Sending guided wp.\n" );
                    mavlink_mission_item_t item;
                    item.param1 = 0;
                    item.param2 = 0;
                    item.param3 = 0;
                    item.param4 = 0;
                    item.x = 40.52904510;
                    item.y = -105.109100341797;
                    item.z = 0.0;
                    item.seq = 1;
                    item.command = 16;
                    item.frame = 3;
                    item.current = 2;  // 2 = guided waypoint,  3 = altitude change only
                    item.autocontinue = 1;

                    comm->SendMissionItem(item);

                    // If balloon disappears (hopefully popped) or time expires, resume to the preprogrammed mission.
                    // After 10 seconds, return to auto
                    struct timeval currTime;
                    gettimeofday(&currTime, NULL);
                    if (currTime.tv_sec > startTime.tv_sec + 2) {
                            printf("-------------------------Returning to AUTO mode.  Continuing with mission item %d.\n", currMissionIndex+1 );
                            comm->SendSetMode(int (AUTO) );
                            comm->SendMissionSetCurrent(currMissionIndex+1);  // pixhawk probably remembers this, also.
                            currState = PREPROGRAMMED_MISSION;
                    }
            }

            break;

    }
    loopCounter++;


}

bool Mission::IsBalloonNearby()
{
  if(pthread_mutex_trylock(&locationLock)) {
    // FINISH...
    if(location.range < 0) {
      return false;
    }
    pthread_mutex_unlock(&locationLock);
  }
    // Get a mutex to check the data structure shared with the computer vision code.
    // Determine if it has found a balloon that is acceptably close.


    return true;
}

bool Mission::CalcBalloonLocation(mavlink_mission_item_t *item)
{
    // FINISH...

    // Get a mutex to check the data structure shared with the computer vision code.
    // Use our current position and attitude, along with the calcuated offsets to
    // the balloon to calculate lat/long/alt of balloon.

    // Return false if the balloon is gone.
    return false;
}

void Mission::TestBalloonMutex()
{
      balloonLocation_t loc;
      if(pthread_mutex_trylock(&locationLock)) {
          loc = location;
          pthread_mutex_unlock(&locationLock);
      }

      printf("In TestBalloonMutex, range = %f, phi = %f, theta = %f, sec = %ld, usec = %ld\n",
        location.range, location.phi, location.theta, (long) location.timestamp.tv_sec, (long) location.timestamp.tv_usec);

    
}