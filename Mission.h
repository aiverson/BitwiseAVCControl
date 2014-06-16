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

class Comm;

class Mission {
public:
    Mission();
    Mission(const Mission& orig);
    virtual ~Mission();

    int GetMissionItemCount();
    int GetReceivedMissionItemCount();
    void SetMissionCount( int missionCount );
    bool StoreMissionItem( mavlink_mission_item_t item);
    void PrintMission();
    void HandleMission(Comm *comm);
private:

    enum MissionState { BOOTING, INITIALIZE, AUTO, GUIDED };

    MissionState currState;
    int loopCounter;
    int missionItemCount;
    int receivedMissionItemCount;
    mavlink_mission_item_t mission[50];

};

#endif	/* MISSION_H */

