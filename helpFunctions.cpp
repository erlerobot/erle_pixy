#include "helpFunctions.h"

void msleep (unsigned int ms) {
    int microsecs;
    struct timeval tv;
    microsecs = ms * 1000;
    tv.tv_sec  = microsecs / 1000000;
    tv.tv_usec = microsecs % 1000000;
    select (0, NULL, NULL, NULL, &tv);
}
