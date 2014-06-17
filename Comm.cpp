/* 
 * File:   Comm.cpp
 * Author: sondra
 * 
 * Created on June 15, 2014, 12:38 PM
 *
 * Connect to and communicate with our copter using MAVLink.
 *
 * The serial interface routines are based on c_uart_interface_example, an
 * example in the public domain by Lorenz Meier, <lm@inf.ethz.ch>
 */

#include "Comm.h"

// Standard includes
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
// Serial includes
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#ifdef __linux
#include <sys/ioctl.h>
#endif

// Latency Benchmarking
#include <sys/time.h>
#include <time.h>

using std::string;
using namespace std;


Comm::Comm() {
    sysid = 1;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
    target_compid = 1;     // pixhawk
    compid = 2;            // me
    silent = false;              ///< Wether console output should be enabled
    verbose = false;             ///< Enable verbose output
    debug = false;               ///< Enable debug functions and output
    lastStatus.packet_rx_drop_count = 0;
    fd = -1;  // not initialized
}

Comm::Comm(const Comm& orig) {
}

Comm::~Comm() {
    ClosePort();
}


int Comm::Startup(int argc, char **argv) {

	/* default values for arguments */
//	char *uart_name = (char*)"/dev/ttyUSB0";
	char *uart_name = (char*)"/dev/ttyACM0";
	int baudrate = 115200;
//        int baudrate = 57600;
	const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [-v/--verbose] [--debug]\n\t\tdefault: -d %s -b %i\n";

	/* read program arguments */
	int i;

	for (i = 1; i < argc; i++) { /* argv[0] is "mavlink" */
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf(commandline_usage, argv[0], uart_name, baudrate);
			return 0;
		}

		/* UART device ID */
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}

		/* baud rate */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}

		/* terminating MAVLink is allowed - yes/no */
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}

		if (strcmp(argv[i], "--debug") == 0) {
			debug = true;
		}
	}

	// SETUP SERIAL PORT

	// Exit if opening port failed
	// Open the serial port.
	if (!silent) printf("Trying to connect to %s.. ", uart_name);
	fflush(stdout);

	int error = OpenPort(uart_name);
	if (error == -1)
	{
		if (!silent) printf("failure, could not open port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}
	if (!silent) printf("Trying to configure %s.. ", uart_name);
	bool setup = SetupPort(baudrate, 8, 1, false, false);
	if (!setup)
	{
		if (!silent) printf("failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}

	int noErrors = 0;
	if (fd == -1 || fd == 0)
	{
		if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	}

	if(fd < 0)
	{
		exit(noErrors);
	}

	// Run indefinitely while the serial loop handles data
	if (!silent) printf("\nREADY, waiting for serial data.\n");

	return 0;
}


/**
 *
 *
 * Returns 0 on success or -1 on error.
 */
int Comm::OpenPort(const char* port)
{

	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		/* Could not open the port. */
		return(-1);
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
	}

        // Create the set of file descriptors used to prevent the read from blocking.
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);

	return (0);
}

bool Comm::SetupPort(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;

	struct termios  config;
	if(!isatty(fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
		return false;
	}
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	                     ONOCR | OFILL | OPOST);

	#ifdef OLCUC
  		config.c_oflag &= ~OLCUC;
	#endif

  	#ifdef ONOEOT
  		config.c_oflag &= ~ONOEOT;
  	#endif

	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	//tcgetattr(fd, &options);

	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;

		// These two non-standard (by the 70'ties ) rates are fully supported on
		// current Debian and Mac OS versions (tested since 2010).
		case 460800:
			if (cfsetispeed(&config, 460800) < 0 || cfsetospeed(&config, 460800) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 921600:
			if (cfsetispeed(&config, 921600) < 0 || cfsetospeed(&config, 921600) < 0)
			{
				fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}

	//
	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}

void Comm::ClosePort()
{
	close(fd);
}

// Not currently supported.
/*
int Comm::SendPing()
{
        struct timeval tv;		  ///< System time

	mavlink_message_t message;
	mavlink_ping_t ping;
	gettimeofday(&tv, NULL);
	ping.time_usec = tv.tv_sec * 1000000 + tv.tv_usec;

	ping.time_usec = 1402800301L * 1e6;
	ping.seq = next_mission;
	ping.target_system    = sysid;
	ping.target_component = target_compid;

	mavlink_msg_ping_encode( sysid, compid, &message, &ping);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);


//	printf(" tv_sec = %d, tv_usec = %d\n", tv.tv_sec, tv.tv_usec);
	printf("------------------------before write - ping usec = %lld, seq = %d, len %d\n", ping.time_usec, ping.seq, len );
	// write packet via serial link
	write(fd, buf, len);
	// wait until all data has been written 
	tcdrain(fd);
	printf("------------------------after write\n");
}
*/

int Comm::SendMissionSetCurrent(int index)
{
	mavlink_message_t message;
	mavlink_mission_set_current_t sc;
	sc.seq              = index;
	sc.target_system    = sysid;
	sc.target_component = target_compid;

	mavlink_msg_mission_set_current_encode( sysid, compid, &message, &sc);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	printf("------------------------ mission set current - index = %d\n", index);
	/* write packet via serial link */
	write(fd, buf, len);
	/* wait until all data has been written */
	tcdrain(fd);
}

int Comm::SendSetMode(int mode)
{
	mavlink_message_t message;

	mavlink_set_mode_t sm;
	sm.custom_mode = mode;    // using 3 for auto, 4 for guided
	sm.target_system = sysid;  // applies to all components, so only need target_system, not target_component.
	sm.base_mode = 1;  // used 5 earlier but 1 should work... // must have bit 1 set for command to work.

	mavlink_msg_set_mode_encode( sysid, compid, &message, &sm);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	printf("------------------------set mode to %d\n", mode );
	/* write packet via serial link */
	write(fd, buf, len);
	/* wait until all data has been written */
	tcdrain(fd);
}

int Comm::SendMissionRequestList() {
	mavlink_message_t message;

	mavlink_mission_request_list_t mrl;
	mrl.target_system = sysid;
        mrl.target_component = target_compid;

	mavlink_msg_mission_request_list_encode( sysid, compid, &message, &mrl);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	printf("------------------------before mission request list write\n" );
	/* write packet via serial link */
	write(fd, buf, len);
	/* wait until all data has been written */
	tcdrain(fd);
	printf("------------------------after mission request list write\n");
}

int Comm::SendMissionRequest(int seq) {
	mavlink_message_t message;

	mavlink_mission_request_t mr;
        mr.seq = seq;
	mr.target_system = sysid;
        mr.target_component = target_compid;

	mavlink_msg_mission_request_encode( sysid, compid, &message, &mr);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	printf("------------------------before mission request for seq %d write\n", seq );
	/* write packet via serial link */
	write(fd, buf, len);
	/* wait until all data has been written */
	tcdrain(fd);
	printf("------------------------after mission request write\n");
}

int Comm::SendMissionItem( mavlink_mission_item_t item) {
	mavlink_message_t message;

        item.target_system    = sysid;
        item.target_component = target_compid;

	mavlink_msg_mission_item_encode( sysid, compid, &message, &item);
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	printf("------------------------send mission item with lat %f, long %f, alt %f, current = %d\n", item.x, item.y, item.z, item.current );
	/* write packet via serial link */
	write(fd, buf, len);
	/* wait until all data has been written */
	tcdrain(fd);
}


/**
 * Read and process messages.
 */
int Comm::ReadMessages(Mission *mission)
{
<<<<<<< HEAD
	printf( "Entering ReadMessages\n");

	// Blocking wait for new data
	{
		//if (debug) printf("Checking for new data on serial port\n");
=======
    int rv;
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    rv = select( fd + 1, &read_fds, NULL, NULL, &timeout);
    if (rv == 0) {
        printf("Timeout trying to read data from Pixhawk.\n");
        return 0;
    } else if (rv == -1) {
        printf("Error trying to read data from Pixhawk.\n");
        return 0;
    } else {
>>>>>>> 46e03c0d895546ee2667a99044150d93aa0b595a
		// Block until data is available, read only one byte to be able to continue immediately
		//char buf[MAVLINK_MAX_PACKET_LEN];
		uint8_t cp;
		mavlink_message_t message;
		mavlink_status_t status;
		uint8_t msgReceived = false;

		if (read(fd, &cp, 1) > 0)
		{
			// Check if a message could be decoded, return the message in case yes
			msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
			if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count)
			{
				if (verbose || debug) printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
				if (debug)
				{
					unsigned char v=cp;
					fprintf(stderr,"%02x ", v);
				}
			}
			lastStatus = status;
		}
		else
		{
			if (!silent) fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
		}

		// If a message could be decoded, handle it
		if(msgReceived)
		{
			//if (verbose || debug) std::cout << std::dec << "Received and forwarded serial port message with id " << static_cast<unsigned int>(message.msgid) << " from system " << static_cast<int>(message.sysid) << std::endl;

			// Do not send images over serial port

			// DEBUG output
			if (debug)
			{
				fprintf(stderr,"Received serial data: ");
				unsigned int i;
				uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
				unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);
				if (messageLength > MAVLINK_MAX_PACKET_LEN)
				{
					fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
				}
				else
				{
					for (i=0; i<messageLength; i++)
					{
						unsigned char v=buffer[i];
						fprintf(stderr,"%02x ", v);
					}
					fprintf(stderr,"\n");
				}
			}

			if (verbose || debug)
				printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

			/* decode and print */


			// For full MAVLink message documentation, look at:
			// https://pixhawk.ethz.ch/mavlink/

			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:           ReceiveMsgHeartbeat(message, mission); break;
				case MAVLINK_MSG_ID_SET_MODE:            ReceiveMsgSetMode(message);          break;
				case MAVLINK_MSG_ID_PING:                ReceiveMsgPing(message);             break;
				case MAVLINK_MSG_ID_STATUSTEXT:          ReceiveMsgStatusText(message);       break;
				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: ReceiveMsgGlobalPosition(message, mission); break;
				case MAVLINK_MSG_ID_ATTITUDE:            ReceiveMsgAttitude(      message, mission); break;
				case MAVLINK_MSG_ID_MISSION_COUNT:       ReceiveMsgMissionCount(  message, mission); break;
				case MAVLINK_MSG_ID_MISSION_CURRENT:     ReceiveMsgMissionCurrent(message, mission); break;
				case MAVLINK_MSG_ID_MISSION_ITEM:        ReceiveMsgMissionItem(   message, mission); break;
				case MAVLINK_MSG_ID_GPS_STATUS:          ReceiveMsgGPSStatus(message);        break;
				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:  ReceiveMsgLocalPositionNED(message); break;
			}

		}
	}
<<<<<<< HEAD
	printf( "Leaving ReadMessages\n");
=======

>>>>>>> 46e03c0d895546ee2667a99044150d93aa0b595a
	return 0;
}

void Comm::ReceiveMsgHeartbeat(mavlink_message_t message, Mission *mission) {
        FlightMode mode;
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&message, &hb);

        printf("Got message HEARTBEAT\n");
        printf("\t type: %d\n", hb.type);
        printf("\t autopilot: %d\n", hb.autopilot);
        printf("\t base_mode: %d\n", hb.base_mode);
        printf("\t custom_mode: %d\n", hb.custom_mode);
        printf("\t system_status: %d\n", hb.system_status);
        printf("\t mavlink_version: %d\n", hb.mavlink_version);
        printf("\n");

        if (hb.base_mode == 81 && hb.custom_mode == 0)
           mode = STABILIZE;
        else if (hb.base_mode == 89 && hb.custom_mode == 3)
            mode = AUTO;
        else if (hb.base_mode == 89 && hb.custom_mode == 4)
            mode = GUIDED;
        else
            mode = OTHER;

        mission->StoreCurrentMode(mode);
}

void Comm::ReceiveMsgSetMode(mavlink_message_t message) {
        mavlink_set_mode_t sm;
        mavlink_msg_set_mode_decode(&message, &sm);

        printf("--------------------------------------------------------------\n");
        printf("Got message SET MODE\n");
        printf("\t custom_mode: %d\n", sm.custom_mode);
        printf("\t target_system: %d\n", sm.target_system);
        printf("\t base_mode: %d\n", sm.base_mode);
        printf("\n");
        printf("--------------------------------------------------------------\n");
}

void Comm::ReceiveMsgPing(mavlink_message_t message) {

        mavlink_ping_t ping;
        mavlink_msg_ping_decode(&message, &ping);

        printf("--------------------------------------------------------------\n");
        printf("Got message PING\n");
        printf("\t time_usec: %lld\n", ping.time_usec);
        printf("\t seq: %d\n", ping.seq);
        printf("\t target_system: %d\n", ping.target_system);
        printf("\t target_component: %d\n", ping.target_component);
        printf("\n");
        printf("--------------------------------------------------------------\n");
}

void Comm::ReceiveMsgStatusText(mavlink_message_t message) {
        mavlink_statustext_t st;
        mavlink_msg_statustext_decode(&message, &st);

        printf("Got message STATUSTEXT\n");
        printf("\t severity: %d\n", st.severity);
        st.text[49] = '\0';
        printf("\t text: %s\n", st.text);
        printf("\n");
}

void Comm::ReceiveMsgGlobalPosition(mavlink_message_t message, Mission *mission) {
        mavlink_global_position_int_t gp;
        mavlink_msg_global_position_int_decode(&message, &gp);

        printf("Got message GLOBAL_POSITION\n");
        printf("\t time_boot_ms: %d\n", gp.time_boot_ms);
        printf("\t lat: %d, %f\n", gp.lat, ((double)gp.lat)/1e7 );
        printf("\t lon: %d, %f\n", gp.lon, ((double)gp.lon)/1e7 );
        printf("\t alt: %d, %f\n", gp.alt, ((double)gp.alt)/1000 );
        printf("\t rel alt: %d, %f\n", gp.relative_alt, ((double)gp.relative_alt)/1000 );
        printf("\t vx: %d, %f\n", gp.vx, ((double)gp.vx)/100 );
        printf("\t vy: %d, %f\n", gp.vy, ((double)gp.vy)/100 );
        printf("\t vz: %d, %f\n", gp.vz, ((double)gp.vz)/100 );
        printf("\t hdg: %d, %f\n", gp.hdg, ((double)gp.hdg)/100 );
        printf("\n");

        mission->StoreGlobalPosition(gp);
}

void Comm::ReceiveMsgAttitude(mavlink_message_t message, Mission *mission) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&message, &attitude);

        printf("Most recent ATTITUDE\n");
        printf("\t time_boot_ms: %d\n", attitude.time_boot_ms);
        printf("\t roll: %f\n", attitude.roll);
        printf("\t pitch: %f\n", attitude.pitch);
        printf("\t yaw: %f\n", attitude.yaw);
        printf("\t rollspeed: %f\n", attitude.rollspeed);
        printf("\t pitchspeed: %f\n", attitude.pitchspeed);
        printf("\t yawspeed: %f\n", attitude.yawspeed);
        printf("\n");

        mission->StoreAttitude(attitude);
}

void Comm::ReceiveMsgLocalPositionNED(mavlink_message_t message) {
        mavlink_local_position_ned_t lp;
        mavlink_msg_local_position_ned_decode(&message, &lp);

        printf("Got message LOCAL_POSITION_NED\n");
        printf("\t time_boot_ms: %d\n", lp.time_boot_ms);
        printf("\t x:  %f\n", lp.x );
        printf("\t y:  %f\n", lp.y );
        printf("\t z:  %f\n", lp.z );
        printf("\t vx: %f\n", lp.vx );
        printf("\t vy: %f\n", lp.vy );
        printf("\t vz: %f\n", lp.vz );
        printf("\n");
}

void Comm::ReceiveMsgMissionCount(mavlink_message_t message, Mission *mission) {
        mavlink_mission_count_t mc;
        mavlink_msg_mission_count_decode(&message, &mc);

        printf("Got message MISSION COUNT\n");
        printf("\t count: %d\n", mc.count);
        printf("\t target_system: %d\n", mc.target_system);
        printf("\t target_component: %d\n", mc.target_component);
        printf("\n");

        mission->SetMissionCount(mc.count);
}

void Comm::ReceiveMsgMissionCurrent(mavlink_message_t message, Mission *mission) {
        mavlink_mission_current_t mc;
        mavlink_msg_mission_current_decode(&message, &mc);

        printf("Got message MISSION CURRENT\n");
        printf("\t seq: %d\n", mc.seq);
        printf("\n");

        mission->StoreCurrentMissionIndex(mc.seq);
}

void Comm::ReceiveMsgMissionItem(mavlink_message_t message, Mission *mission) {
        mavlink_mission_item_t mi;
        mavlink_msg_mission_item_decode(&message, &mi);

        printf("Got message MISSION ITEM\n");
        printf("\t seq: %d\n", mi.seq);
        printf("\t param1: %f\n", mi.param1);
        printf("\t param2: %f\n", mi.param2);
        printf("\t param3: %f\n", mi.param3);
        printf("\t param4: %f\n", mi.param4);
        printf("\t x: %f\n", mi.x);
        printf("\t y: %f\n", mi.y);
        printf("\t z: %f\n", mi.z);
        printf("\t command: %d\n", mi.command);
        printf("\t target_system: %d\n", mi.target_system);
        printf("\t target_component: %d\n", mi.target_component);
        printf("\t frame: %d\n", mi.frame);
        printf("\t current: %d\n", mi.current);
        printf("\t autocontinue: %d\n", mi.autocontinue);
        printf("\n");

        mission->StoreMissionItem(mi);
}

void Comm::ReceiveMsgGPSStatus(mavlink_message_t message) {
        mavlink_gps_status_t gs;
        mavlink_msg_gps_status_decode(&message, &gs);

        printf("Got message GPS STATUS\n");
        printf("\t satellites_visible: %d\n", gs.satellites_visible);
        printf("\n");
}
