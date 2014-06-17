/* 
 * File:   Mission.h
 * Author: sondra
 *
 * Created on June 15, 2014, 11:05 AM
 */

#ifndef MISSION_H
#define	MISSION_H

#include <common/mavlink.h>

#include "Comm.h"
#include "BalloonLocation.h"
#include <sys/time.h>
#include <time.h>

class Comm;

enum FlightMode { STABILIZE = 0, AUTO = 3, GUIDED = 4, OTHER = 99};

class Mission {
public:
    Mission();
    Mission(const Mission& orig);
    virtual ~Mission();

    void SetMissionCount( int missionCount );
    bool StoreMissionItem( mavlink_mission_item_t item);
    void StoreGlobalPosition( mavlink_global_position_int_t pos );
    void StoreCurrentMissionIndex( int index );
    void StoreCurrentMode(FlightMode mode);
    void PrintMission();
    void PrintGlobalPosition();
    void HandleMission(Comm *comm);
private:

    enum MissionState { BOOTING, INITIALIZE, PREPROGRAMMED_MISSION, SEARCHING_FOR_BALLOON, CHASING_BALLOON };

    MissionState currState;
    int loopCounter;
    int missionItemCount;
    int receivedMissionItemCount;
    mavlink_mission_item_t mission[50];

    mavlink_global_position_int_t globalPosition;  // values are scaled integers
    int  currMissionIndex;
    int  missionIndexWhenReturnToAuto;
    struct timeval startTime;

    FlightMode currFlightMode;

    bool IsBalloonNearby();
    bool CalcBalloonLocation(mavlink_mission_item_t *item);
    void TestBalloonMutex();

};

#endif	/* MISSION_H */

