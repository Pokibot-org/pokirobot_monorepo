#ifndef NAV_H
#define NAV_H

#include <zephyr/kernel.h>
#include "pokibot/poktypes.h"
#include <stdint.h>

#define NAV_EVENT_DESTINATION_REACHED BIT(0)
#define NAV_EVENT_TIMEOUT             BIT(1)
#define NAV_EVENT_CANCELED            BIT(2)
#define ALL_NAV_EVENTS                (NAV_EVENT_DESTINATION_REACHED | NAV_EVENT_TIMEOUT | NAV_EVENT_CANCELED)

enum nav_obstacle_type {
    NAV_OBSTACLE_TYPE_CIRCLE,
    NAV_OBSTACLE_TYPE_RECTANGLE,
};

struct nav_obstacle_circle {
    point2_t point;
    float radius;
};

struct nav_obstacle_rectangle {
    point2_t point;
    float width;
    float height;
};

struct nav_obstacle {
    enum nav_obstacle_type type;
    union {
        struct nav_obstacle_circle circle;
        struct nav_obstacle_rectangle rectangle;
    } data;
};

#define NAV_PLANAR_VMAX_DEFAULT    400.0f
#define NAV_ANGULAR_VMAX_DEFAULT   (0.7f * M_PI)

int nav_set_pos(const pos2_t pos);
int nav_set_break(bool status);
int nav_go_to(const pos2_t pos, k_timeout_t timeout);
int nav_go_to_direct(const pos2_t pos, k_timeout_t timeout);
int nav_set_speed(float planar_vmax, float angular_vmax);
void nav_cancel(void);
void nav_wait_events(uint32_t *events);
void nav_clear_obstacles(void);
void nav_register_obstacle(struct nav_obstacle *obs);

#endif
