/* 
 * File:   Mission.cpp
 * Author: sondra
 * 
 * Created on June 15, 2014, 11:05 AM
 */

#include <stdio.h>
#include "Mission.h"
#include "mavlink/include/pixhawk/pixhawk.h"

Mission::Mission() {
    currState = INITIALIZE;
  //currState = PREPROGRAMMED_MISSION;
    receivedMissionItemCount = 0;
    missionItemCount = -1;
    loopCounter = 0;
    currMissionIndex = 0;
    numIterationsWithoutSeeingBalloon = 0;
    gettimeofday(&lastMissionUpdateTime, NULL);
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
    PrintAttitude();

}

void Mission::StoreGlobalPosition( mavlink_global_position_int_t pos ) {
    globalPosition = pos;
}

void Mission::StoreAttitude( mavlink_attitude_t newAttitude ) {
    attitude = newAttitude;
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

void Mission::PrintAttitude() {
        printf("Most recent ATTITUDE\n");
        printf("\t time_boot_ms: %d\n", attitude.time_boot_ms);
        printf("\t roll: %f\n", attitude.roll);
        printf("\t pitch: %f\n", attitude.pitch);
        printf("\t yaw: %f\n", attitude.yaw);
        printf("\t rollspeed: %f\n", attitude.rollspeed);
        printf("\t pitchspeed: %f\n", attitude.pitchspeed);
        printf("\t yawspeed: %f\n", attitude.yawspeed);
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

long Mission::GetTimeDelta(struct timeval timea, struct timeval timeb) {
  return 1000000 * (timeb.tv_sec - timea.tv_sec) +
    (int(timeb.tv_usec) - int(timea.tv_usec));
}

void Mission::HandleMission(Comm *comm) {
    struct timeval currTime;
    gettimeofday(&currTime, NULL);

    long timeDelta = GetTimeDelta(lastMissionUpdateTime, currTime);
    printf("timeDelta = %ld\n", timeDelta);

    if (timeDelta > TIME_BETWEEN_UPDATES) {
        lastMissionUpdateTime = currTime;
        comm->SendMsgHeartbeat();

        switch (currState) {
            case INITIALIZE:
                printf("-----------------------INITIALIZE\n");
                // Get the Mission from the pixhawk.
                printf("-------------------------In INITIALIZE, missionItemCount = %d, receivedMissionItemCount = %d\n",
                        missionItemCount, receivedMissionItemCount);

                // In response to the MAVLink message MISSION_REQUEST_LIST, the pixhawk only returns the count of
                // mission items.  It doesn't actually return the list.  Each item must be requested separately.
                if (missionItemCount == -1) {
                    comm->SendMissionRequestList();
                    // Let's also disable the failsafe timeout.  0 = disable.
                    // I'm not sure about which param_type to use, but UINT8
                    // seems reasonable for a param which can only take the values 0, 1, 2
                    char paramName[16] = "FS_GCS_ENABLE";
                    comm->SendMsgParamSet(paramName, MAV_PARAM_TYPE_UINT8, 0);  // disable failsafe
                }

                    // Request the mission items one at a time.  Repeating requests, if necessary.
                else if (receivedMissionItemCount < missionItemCount)
                    comm->SendMissionRequest(receivedMissionItemCount);

                else if (receivedMissionItemCount >= missionItemCount) {
                    currState = PREPROGRAMMED_MISSION; // We have the whole list.

                    printf("-------------------------Switching from state INITIALIZE to PREPROGRAMMED_MISSION\n");
                }

                break;

            case PREPROGRAMMED_MISSION:
                printf("-----------------------PREPROGRAMMED_MISSION\n");

                // We are in a preprogrammed part of the mission.  Watch for our special
                // indicators that a balloon should be near.

                // May want to insert LOITER commands (or some other flag) to indicate we should
                // start searching for balloons.

                // Store time that we started searching for a balloon.  Switch to mode SEARCHING_FOR_BALOON.

		printf("--------------------currFlightMode = %d, want AUTO = %d, currMissionIndex = %d, command = %d, want loiter %d\n",
			currFlightMode, AUTO, currMissionIndex, mission[currMissionIndex].command, MAV_CMD_NAV_LOITER_TIME);

                if (currFlightMode == AUTO && mission[currMissionIndex].command == MAV_CMD_NAV_LOITER_TIME) { // cmd id is 19
                    printf("--------------------------------In Loiter mode, switch to SEARCHING_FOR_BALLOON\n");
                    currState = SEARCHING_FOR_BALLOON;
                    missionIndexWhenReturnToAuto = currMissionIndex + 1;
                    gettimeofday(&startSearchingForBalloonTime, NULL);
                }

                break;

            case SEARCHING_FOR_BALLOON:
                printf("-----------------------SEARCHING_FOR_BALLOON\n");
                // We are still in auto mode flying a preprogrammed mission, but we are
                // also expecting to find a balloon in this segment of the mission.

                // If we find a reasonably close balloon, switch to guided flight mode and go pop it -
                // (i.e. switch to CHASING_BALLOON state.)

                // If we do not find a balloon in a reasonable amount of time, switch
                // back to PREPROGRAMMED_MISSION mode.
                if (currTime.tv_sec > startSearchingForBalloonTime.tv_sec + MAX_SECONDS_TO_SEARCH_FOR_BALLOON ||
                        currMissionIndex > missionIndexWhenReturnToAuto) {
                    comm->SendSetMode(int (AUTO));
                    if (currMissionIndex < missionIndexWhenReturnToAuto) {
                        comm->SendMissionSetCurrent(missionIndexWhenReturnToAuto);
                        printf("-------------------------Can't find balloon.  Returning to PREPROGRAMMED_MISSION mode.  Switching from current mission item %d to %d.\n", currMissionIndex, missionIndexWhenReturnToAuto);
                    }
                    printf("-------------------------Can't find balloon.  Returning to PREPROGRAMMED_MISSION mode.  Continuing with current mission item %d.\n", currMissionIndex);

                    currState = PREPROGRAMMED_MISSION;

                } else if (currFlightMode == AUTO) {

                    if (IsBalloonNearby()) {

                        printf("-------------------------Found a balloon.  Requesting mode change to GUIDED.\n");
                        comm->SendSetMode(int (GUIDED));
                        gettimeofday(&startChasingBalloonTime, NULL);

                        currState = CHASING_BALLOON;
                        numIterationsWithoutSeeingBalloon = 0;
                    }
                }

                break;
                
            case CHASING_BALLOON:
                printf("-----------------------CHASING_BALLOON\n");

                // Periodically compare newly calculated balloon location with last balloon location.
                // If altitude has changed, send waypoint with current = 3.  Changes altitude only
                // If balloon location changed, send waypoint with current = 2.  (i.e., a guided waypoint)


                // If balloon disappears (hopefully popped) or time expires, resume the preprogrammed mission,
                // (i.e., flight mode AUTO and state PREPROGRAMMED_MISSION). 


                if (currFlightMode != GUIDED) {
                    printf("CurrFlightMode should be GUIDED, but is %d.  Send request again.\n", currFlightMode);
                    comm->SendSetMode(int (GUIDED));

                } else {

                    mavlink_mission_item_t newCommand;
                    bool stillTrackingBalloon = CalcBalloonLocation(&newCommand);

                    if (!stillTrackingBalloon) {
                        numIterationsWithoutSeeingBalloon++;
                        printf("------------------numIterationsWithoutSeeingBalloon = %d\n", numIterationsWithoutSeeingBalloon);
                    }

                    if (numIterationsWithoutSeeingBalloon > MAX_ITERATIONS_WITHOUT_FINDING_BALLOON ||
                            (currTime.tv_sec > startChasingBalloonTime.tv_sec + MAX_SECONDS_TO_CHASE_BALLOON)) {
                        // If balloon disappears (hopefully popped) or time expires, resume to the preprogrammed mission.
                        printf("-------------------------Balloon is gone or time expired.  Returning to AUTO mode.  Continuing with mission item %d.\n", currMissionIndex + 1);
                        printf("-------------------------numIterationsWithoutFindingBalloon %d, max iterations = %d.\n", numIterationsWithoutSeeingBalloon, MAX_ITERATIONS_WITHOUT_FINDING_BALLOON);
                        comm->SendSetMode(int (AUTO));
                        comm->SendMissionSetCurrent(missionIndexWhenReturnToAuto); 
                        currState = PREPROGRAMMED_MISSION;

                    } else {

                        printf("*********(%f, %f, %f)\n", newCommand.x, newCommand.y, newCommand.z);
                        PrintGlobalPosition();

                        numIterationsWithoutSeeingBalloon = 0;  // possibly only reset this if waypoint is valid.

                        if (IsWaypointReasonable(&newCommand)) {
                            newCommand.param1 = 0;
                            newCommand.param2 = 0;
                            newCommand.param3 = 0;
                            newCommand.param4 = 0;

                            newCommand.seq = 1;
                            newCommand.command = 16;
                            newCommand.frame = 3;
                            newCommand.current = 2; // 2 = guided waypoint
                            newCommand.autocontinue = 1;
                            comm->SendMissionItem(newCommand);
                            printf("-------------------------Sending guided wp.\n");
                        } else {
                            printf("-------------------------Not sending unreasonable guided wp.\n");
                        }

                    }
                }
                break;

            case SWITCHING_BACK_TO_AUTO:
                // Since it is critical that we can successfully switch back to AUTO,
                // we have a separate state to make sure the switch happens.  Keep issuing the requests until
                // until we are notified that the state successfully changed and we are running
                // the desired command.
                if (currFlightMode == AUTO && currMissionIndex >= missionIndexWhenReturnToAuto) {
                    currState = PREPROGRAMMED_MISSION;
                    
                } else {
                    printf("CurrFlightMode should be AUTO, but is %d.  Send request again.\n", currFlightMode);
                    comm->SendSetMode(int (AUTO));
                    if (currMissionIndex < missionIndexWhenReturnToAuto) {
                        comm->SendMissionSetCurrent(missionIndexWhenReturnToAuto);
                        printf("-------------------------SWITCHING_BACK_TO_AUTO.  Switching from current mission item %d to %d.\n", currMissionIndex, missionIndexWhenReturnToAuto);
                    }
                }

                break;
        }
    }
}

bool Mission::IsWaypointReasonable(mavlink_mission_item_t *command) {
    const float MIN_REASONABLE_LAT = 40.0;
    const float MAX_REASONABLE_LAT = 41.0;
    const float MIN_REASONABLE_LON = -105.6;
    const float MAX_REASONABLE_LON = -104.6;
    const float MIN_REASONABLE_ALT = 0.5;
    const float MAX_REASONABLE_ALT = 5.0;

    // Keep the position requests within a reasonable limit.
    if (command->x < MIN_REASONABLE_LAT || command->x > MAX_REASONABLE_LAT ||
        command->y < MIN_REASONABLE_LON || command->y > MAX_REASONABLE_LON ||
        command->z < MIN_REASONABLE_ALT || command->z > MAX_REASONABLE_ALT) {
        printf("------------------------lat/lon/alt is out of range.  Request was %f, %f, %f\n", command->x, command->y, command->z);
        return false;
    }

    return true;
}

bool Mission::IsBalloonNearby() {

    float range = -1.0;

//    printf("get mutex for IsBalloonNearby\n");
    // Get a mutex to check the data structure shared with the computer vision code.
//    if (pthread_mutex_trylock(&locationLock)) {
    pthread_mutex_lock(&locationLock);
//    	printf("got trylock mutex for IsBalloonNearby\n");
        range = location.range;
        pthread_mutex_unlock(&locationLock);
//    }
//    printf("release mutex for IsBalloonNearby\n");

    // Determine if it has found a balloon that is acceptably close.
    if(range >= 0 && range < MAX_DISTANCE_TO_BALLOON) {
        printf("------------------There's a balloon nearby!  It is %f meters away.\n", range);
        return true;
    }

    printf("------------------Balloon is not in range.  Range = %f\n", range);
    return false;
}

bool Mission::CalcBalloonLocation(mavlink_mission_item_t *item)
{
  //printf("Entering CalcBalloonLocation\n");

    // Get a mutex to check the data structure shared with the computer vision code.
  pthread_mutex_lock(&locationLock);
//  printf("CalcBalloonLocation locked mutex\n");
    // Use our current position and attitude, along with the calcuated offsets to
  // the balloon to calculate lat/long/alt of balloon.
  const double m1 = 111132.92;
  const double m2 = -559.82;
  const double m3 = 1.175;
  const double m4 = 0.0023;
  const double p1 = 111412;
  const double p2 = -93.5;
  const double p3 = 0.118;

  double lat = globalPosition.lat*1.0e-7;
  double lon = globalPosition.lon*1.0e-7;
  double latrad = lat*3.1415926/180;
  double lonrad = lon*3.1415926/180;
  double alt = globalPosition.relative_alt/1000.0;
  double pitch = attitude.pitch;
  double yaw = attitude.yaw;
  double rho = location.range;
  double phi = location.phi;
  double theta = location.theta;
  pthread_mutex_unlock(&locationLock);

  printf("In CalcBalloonLocation, theta = %f, phi = %f, range = %f\n", theta, phi, rho);

  // Return false if the balloon is gone.
  if (rho < 0 || rho > MAX_DISTANCE_TO_BALLOON)
      return false;
  
  double latlen = m1+m2*cos(2*latrad)+m3*cos(4*latrad)+m4*cos(6*latrad);
  double lonlen = p1*cos(latrad)+p2*cos(3*latrad)+p3*cos(latrad);

  double newlat = rho*cos(phi+pitch)*sin(theta+yaw)/latlen+lat;
  double newlon = rho*cos(phi+pitch)*cos(theta+yaw)/lonlen+lon;
  double newalt = rho*sin(phi+pitch)+alt;
  
  item->x = newlat;
  item->y = newlon;
  item->z = newalt;
//  printf("leaving CalcBalloonLocation\n");

  return true;
}

void Mission::TestBalloonMutex()
{
      balloonLocation_t loc;
      //printf("Before requesting mutex.\n");
	//if(pthread_mutex_trylock(&locationLock)) {
	pthread_mutex_lock(&locationLock);
          loc = location;
          pthread_mutex_unlock(&locationLock);
	  //}
	  //printf("After releasing mutex.\n");

      printf("In TestBalloonMutex, range = %f, phi = %f, theta = %f, sec = %ld, usec = %ld\n",
        location.range, location.phi, location.theta, (long) location.timestamp.tv_sec, (long) location.timestamp.tv_usec);

    
}
