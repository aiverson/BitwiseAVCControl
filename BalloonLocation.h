#ifndef BALLOON_LOCATION_H
#define BALLOON_LOCATION_H

#include <pthread.h>

typedef struct balloonLocation {
  double range;
  double phi;
  double theta;
  struct timeval timestamp;
} balloonLocation_t;

pthread_mutex_t locationLock = PTHREAD_MUTEX_INITIALIZER;

balloonLocation_t location;

#endif
