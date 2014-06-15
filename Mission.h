/* 
 * File:   Mission.h
 * Author: sondra
 *
 * Created on June 15, 2014, 11:05 AM
 */

#ifndef MISSION_H
#define	MISSION_H

#include <common/mavlink.h>

class Mission {
public:
    Mission();
    Mission(const Mission& orig);
    virtual ~Mission();

    bool StoreMissionItem( mavlink_mission_item_t item);
private:

    int reportedMissionItemCount;
    int originalMissionItemCount;
    mavlink_mission_item_t originalMission[50];

};

#endif	/* MISSION_H */

