#ifndef BALLOON_LOCATION_H
#define BALLOON_LOCATION_H

#include <sys/time.h>
#include <pthread.h>

typedef struct balloonLocation {
  double range;
  double phi;
  double theta;
  struct timeval timestamp;
} balloonLocation_t;

extern pthread_mutex_t locationLock;

extern balloonLocation_t location;

#endif
