#ifndef POKTYPES_H
#define POKTYPES_H

#include <math.h>

#ifndef MAXFLOAT
#define MAXFLOAT 3.40282347e+38F
#endif

#if !(defined __USE_MISC || defined __USE_XOPEN)
#define M_E        2.7182818284590452354f
#define M_LOG2E    1.4426950408889634074f
#define M_LOG10E   0.43429448190325182765f
#define M_LN2      0.693147180559945309417f
#define M_LN10     2.30258509299404568402f
#define M_PI       3.14159265358979323846f
#define M_PI_2     1.57079632679489661923f
#define M_PI_4     0.78539816339744830962f
#define M_1_PI     0.31830988618379067154f
#define M_2_PI     0.63661977236758134308f
#define M_2_SQRTPI 1.12837916709551257390f
#define M_SQRT2    1.41421356237309504880f
#define M_SQRT1_2  0.70710678118654752440f
#define M_SQRT3    1.73205080756887729352f
#endif

typedef struct point2 {
    float x;
    float y;
} point2_t;

typedef struct vec2 {
    float dx;
    float dy;
} vec2_t;

typedef struct pos2 {
    float x;
    float y;
    float a;
} pos2_t;

typedef struct vel2 {
    float vx;
    float vy;
    float w;
} vel2_t;


#endif
