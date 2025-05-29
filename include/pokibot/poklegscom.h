#ifndef POKLEGSCOM_H
#define POKLEGSCOM_H
#include <pokibot/lib/poktocol.h>
#include <pokibot/poktypes.h>
#include <stdint.h>

int poklegscom_set_pos(const pos2_t *pos);
int poklegscom_set_waypoints(const pos2_t *waypoints, size_t nb_waypoints);
int poklegscom_get_pos(pos2_t *pos);
int poklegscom_get_dir(float *dir);
int poklegscom_set_break(bool state);
int poklegscom_set_speed(float planar_vmax, float angular_vmax);

#endif
