#ifndef POKDEFS_H
#define POKDEFS_H
#include <pokibot/lib/pokutils.h>

#define BOARD_MIN_X    -1.5f
#define BOARD_MAX_X    1.5f
#define BOARD_CENTER_X ((BOARD_MIN_X + BOARD_MAX_X) / 2)
#define BOARD_MIN_Y    0.0f
#define BOARD_MAX_Y    2.0f
#define BOARD_CENTER_Y ((BOARD_MIN_Y + BOARD_MAX_Y) / 2)

#define ROBOT_RADIUS   0.19f

#define OPPONENT_ROBOT_MAX_RADIUS   0.191f // 1.2/(2*pi)

#define TAG_POLE_MIN_RADIUS (0.07f/2)

#define LIDAR_STOP_DISTANCE (ROBOT_RADIUS + OPPONENT_ROBOT_MAX_RADIUS - TAG_POLE_MIN_RADIUS + 0.05f)
#define LIDAR_STOP_ANGLE    (M_PI/2)

#endif
