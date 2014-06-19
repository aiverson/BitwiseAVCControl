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

const float MAX_DISTANCE_TO_BALLOON = 20.0;  // meters
const long MAX_SECONDS_TO_CHASE_BALLOON = 10;
const long MAX_SECONDS_TO_SEARCH_FOR_BALLOON = 20;
const long ITERATIONS_PER_SECOND = 10;
const long TIME_BETWEEN_UPDATES = 1000000 / ITERATIONS_PER_SECOND;
const int  MAX_ITERATIONS_WITHOUT_FINDING_BALLOON = ITERATIONS_PER_SECOND * 2;

class Mission {
public:
    Mission();
    Mission(const Mission& orig);
    virtual ~Mission();

    void SetMissionCount( int missionCount );
    bool StoreMissionItem( mavlink_mission_item_t item);
    void StoreGlobalPosition( mavlink_global_position_int_t pos );
    void StoreAttitude( mavlink_attitude_t newAttitude );
    void StoreCurrentMissionIndex( int index );
    void StoreCurrentMode(FlightMode mode);
    void PrintMission();
    void PrintGlobalPosition();
    void PrintAttitude();
    void HandleMission(Comm *comm);
private:

    enum MissionState { INITIALIZE, PREPROGRAMMED_MISSION, SEARCHING_FOR_BALLOON, CHASING_BALLOON };

    MissionState currState;
    int loopCounter;
    int missionItemCount;
    int receivedMissionItemCount;
    mavlink_mission_item_t mission[50];

    mavlink_global_position_int_t globalPosition;  // Most recently received position.  Values are scaled integers.
    mavlink_attitude_t attitude;                   // Most recently received attitude.
    
    int  currMissionIndex;
    int  missionIndexWhenReturnToAuto;
    struct timeval lastMissionUpdateTime;
    struct timeval startSearchingForBalloonTime;
    struct timeval startChasingBalloonTime;

    int numIterationsWithoutSeeingBalloon;

    FlightMode currFlightMode;

    long GetTimeDelta(struct timeval timea, struct timeval timeb);
    bool IsBalloonNearby();
    bool CalcBalloonLocation(mavlink_mission_item_t *item);
    void TestBalloonMutex();

};

#endif	/* MISSION_H */

