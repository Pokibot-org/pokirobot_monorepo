#ifndef POKUTILS_H
#define POKUTILS_H

#include <zephyr/kernel.h>
#include <pokibot/poktypes.h>
#include <math.h>

#define RAD_TO_DEG(_rad) ((_rad) * 180.0f / M_PI)
#define DEG_TO_RAD(_deg) ((_deg) / 180.0f * M_PI)

#define SIGNF(_val)     (signbit(_val) ? -1.0f : 1.0f)
#define NEG_SQRTF(_val) (SIGNF(_val) * sqrtf(fabsf(_val)))

#define CONVERT_POS2_TO_POINT2(pos)                                                                \
    (point2_t)                                                                                     \
    {                                                                                              \
        .x = (pos).x, .y = (pos).y                                                                 \
    }
#define CONVERT_POINT2_TO_POS2(point, angle)                                                       \
    (pos2_t)                                                                                       \
    {                                                                                              \
        .x = (point).x, .y = (point).y, .a = (angle)                                               \
    }

#define POS2_COMPARE(pos_a, operator, pos_b)                                                       \
    (((pos_a).x operator(pos_b).x) && ((pos_a).y operator(pos_b).y) &&                             \
     ((pos_a).a operator(pos_b).a))

#define POINT2_COMPARE(a, operator, b) (((a).x operator(b).x) && ((a).y operator(b).y))

vec2_t point2_diff(const point2_t terminal, const point2_t initial);
vec2_t vec2_normalize(vec2_t vec);
float vec2_abs(vec2_t vec);
float vec2_dot(vec2_t a, vec2_t b);
float vec2_angle(vec2_t a, vec2_t b);
float vec2_distance(point2_t a, point2_t b);

pos2_t pos2_diff(const pos2_t target, const pos2_t orig);
pos2_t pos2_add(const pos2_t a, const pos2_t b);
pos2_t pos2_abs(const pos2_t a);
float angle_normalize(float a);

#endif // POKUTILS_H
