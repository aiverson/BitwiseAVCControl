/** This example is public domain. */


/**
 * @file
 *   @brief The serial interface process
 *
 *   This process connects any external MAVLink UART device and prints data
 *
 *   @author Lorenz Meier, <lm@inf.ethz.ch>
 *
 */

#include <common/mavlink.h>

#include "Comm.h"



int main(int argc, char **argv) {

    Comm comm;
    comm.Startup(argc, argv);

    while (true) {
        comm.ReadMessages();
        comm.SendSomeStuff();

    }


}


