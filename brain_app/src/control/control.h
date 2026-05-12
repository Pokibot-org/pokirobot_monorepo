#ifndef CONTROL_H
#define CONTROL_H
#include <pokibot/poktypes.h>
#include <stdint.h>
#include <stdbool.h>

int control_set_pos(const pos2_t *pos);
int control_set_waypoints(const pos2_t *waypoints, size_t nb_waypoints);
int control_get_pos(pos2_t *pos);
int control_get_dir(float *dir);
int control_set_break(bool state);
int control_set_speed(float planar_vmax, float angular_vmax);

#endif
