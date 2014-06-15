/* 
 * File:   Comm.h
 * Author: sondra
 *
 * Created on June 15, 2014, 12:38 PM
 */

#ifndef COMM_H
#define	COMM_H

#include <common/mavlink.h>

class Comm {
public:
    Comm();
    Comm(const Comm& orig);
    virtual ~Comm();

    int  Startup(int argc, char **argv);

    int  OpenPort(const char* port);
    bool SetupPort(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
    void ClosePort();

    int SendPing();
    int SendNextCommand();
    int SendSetMode();
    int SendSomeStuff();
    int ReadMessages();
    int SendMissionRequestList();
    int SendMissionRequest(int itemToRequest);



private:
    int sysid;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
    int compid;
    bool silent;              ///< Whether console output should be enabled
    bool verbose;             ///< Enable verbose output
    bool debug;               ///< Enable debug functions and output
    int fd;             /* File descriptor for the port */

    int missionCount;
    int missionItemsReceived;
    
    char buf[300];
    unsigned loopcounter;
    int next_mission;
    mavlink_status_t lastStatus;

    void ReceiveMsgHeartbeat(mavlink_message_t message);
    void ReceiveMsgSetMode(mavlink_message_t message);
    void ReceiveMsgPing(mavlink_message_t message);
    void ReceiveMsgStatusText(mavlink_message_t message);
    void ReceiveMsgGlobalPosition(mavlink_message_t message);
    void ReceiveMsgLocalPositionNED(mavlink_message_t message);
    void ReceiveMsgMissionCount(mavlink_message_t message);
    void ReceiveMsgMissionCurrent(mavlink_message_t message);
    void ReceiveMsgMissionItem(mavlink_message_t message);
    void ReceiveMsgGPSStatus(mavlink_message_t message);

};
#endif	/* COMM_H */


