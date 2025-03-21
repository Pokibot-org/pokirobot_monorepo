#include <math.h>
#include <zephyr/kernel.h>
#include <pokibot/lib/pokutils.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pokutils);

vec2_t point2_diff(const point2_t terminal, const point2_t initial)
{
    vec2_t delta = {
        .dx = terminal.x - initial.x,
        .dy = terminal.y - initial.y,
    };
    return delta;
}

vec2_t vec2_normalize(vec2_t vec)
{
    float length = vec2_abs(vec);
    if (length > 0) {
        vec.dx /= length;
        vec.dy /= length;
    }
    return vec;
}

float vec2_abs(vec2_t vec)
{
    return sqrtf(vec.dx * vec.dx + vec.dy * vec.dy);
}

float vec2_dot(vec2_t a, vec2_t b)
{
    return a.dx * b.dx + a.dy * b.dy;
}

float vec2_angle(vec2_t a, vec2_t b)
{
    return acosf(vec2_dot(a, b) / (vec2_abs(a) * vec2_abs(b)));
}

float vec2_distance(point2_t a, point2_t b)
{
    return vec2_abs(point2_diff(a, b));
}

pos2_t pos2_diff(const pos2_t terminal, const pos2_t initial)
{
    pos2_t delta = {
        .x = terminal.x - initial.x,
        .y = terminal.y - initial.y,
        .a = terminal.a - initial.a,
    };
    return delta;
}

pos2_t pos2_add(const pos2_t a, const pos2_t b)
{
    pos2_t c = {
        .x = a.x + b.x,
        .y = a.y + b.y,
        .a = a.a + b.a,
    };
    return c;
}

pos2_t pos2_abs(const pos2_t a)
{
    pos2_t c = {
        .x = fabsf(a.x),
        .y = fabsf(a.y),
        .a = fabsf(a.a),
    };
    return c;
}

float angle_normalize(float a)
{
    a = fmodf(a + M_PI, 2 * M_PI); // Ensure it's within [-2PI, 2PI]
    if (a < 0)
        a += 2 * M_PI; // Make it positive in [0, 2PI]
    return a - M_PI;   // Shift to [-PI, PI]
}
