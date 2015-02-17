#ifndef __CONSTS_H
#define __CONSTS_H

// Constants for map and histogramm
#define MAP_RES 4
#define MAP_COUNT 60
#define HIST_RES 1
#define HIST_COUNT 64
#define RPM 30
#define HIST_MAX_SUM (HIST_COUNT*360*360)
#define arenalon 2
#define arenawidth 1
#define arenaheight 0
#define arenares 0.1
#define arencount (int)(2/0.2+1)
// Constants table to convert a direction to a pair of dx/dy
extern const int dirToDxDy4[4][2];
extern const int dirToDxDy8[8][2];

#endif
