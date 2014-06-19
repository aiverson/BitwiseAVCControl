/* 
 * File:   Comm.h
 * Author: sondra
 *
 * Created on June 15, 2014, 12:38 PM
 */

#ifndef COMM_H
#define	COMM_H

#include <common/mavlink.h>
#include <sys/select.h>

#include "Mission.h"

class Mission;

class Comm {
public:
    Comm();
    Comm(const Comm& orig);
    virtual ~Comm();

    int  Startup(int argc, char **argv);
    int  OpenPort(const char* port);
    bool SetupPort(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
    void ClosePort();

    int SendMsgHeartbeat();
    int SendMsgParamSet(char *param_id, uint8_t param_type, float param_value);
    int SendMissionSetCurrent(int index);
    int SendSetMode(int mode);
    int SendMissionRequestList();
    int SendMissionRequest(int itemToRequest);
    int SendMissionItem( mavlink_mission_item_t item);
    
    int ReadMessages(Mission *mission);



private:
    int sysid;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
    int compid;
    int target_compid;
    bool silent;              ///< Whether console output should be enabled
    bool verbose;             ///< Enable verbose output
    bool debug;               ///< Enable debug functions and output
    int fd;             /* File descriptor for the port */
    fd_set read_fds;

    char buf[300];
    mavlink_status_t lastStatus;

    void ReceiveMsgHeartbeat(mavlink_message_t message, Mission *mission);
    void ReceiveMsgParamValue(mavlink_message_t message);
    void ReceiveMsgSetMode(mavlink_message_t message);
    void ReceiveMsgStatusText(mavlink_message_t message);
    void ReceiveMsgGlobalPosition(mavlink_message_t message, Mission *mission);
    void ReceiveMsgAttitude(      mavlink_message_t message, Mission *mission);
    void ReceiveMsgMissionCount(  mavlink_message_t message, Mission *mission);
    void ReceiveMsgMissionCurrent(mavlink_message_t message, Mission *mission);
    void ReceiveMsgMissionItem(   mavlink_message_t message, Mission *mission);
    void ReceiveMsgGPSStatus(mavlink_message_t message);

    // Not being sent
    void ReceiveMsgPing(mavlink_message_t message);
    void ReceiveMsgLocalPositionNED(mavlink_message_t message);
};
#endif	/* COMM_H */


