#ifndef POKI_TRIGO_H
#define POKI_TRIGO_H

/***
 * CONFIGURATION SECTION
 ***/ 
#ifndef POKI_SIN_COS_LEVEL
/*
 * PRECISION FOR SINE / COSINE COMPUTATIONS
 * POKI_SIN_COS_LEVEL = 3  min precision: 13.57 bits
 * POKI_SIN_COS_LEVEL = 4  min precision: 19.95 bits
 * POKI_SIN_COS_LEVEL = 5  min precision: 21.12 bits
 */
#define POKI_SIN_COS_LEVEL 5
#endif

#ifndef POKI_TAN_LEVEL
/*
 * PRECISION FOR TAN / COTAN COMPUTATIONS
 * POKI_TAN_LEVEL = 3  min precision: 8.26 bits
 * POKI_TAN_LEVEL = 4  min precision: 12.55 bits
 * POKI_TAN_LEVEL = 5  min precision: 15.83 bits
 * POKI_TAN_LEVEL = 6  min precision: 18.89 bits
 * POKI_TAN_LEVEL = 7  min precision: 21.41 bits
 */
#define POKI_TAN_LEVEL 7
#endif

#ifndef POKI_ATAN_LEVEL
/*
 * PRECISION FOR ATAN / ATAN2 / ACOTAN COMPUTATIONS
 * POKI_ATAN_LEVEL = 3  min precision: 10.34 bits
 * POKI_ATAN_LEVEL = 4  min precision: 13.32 bits
 * POKI_ATAN_LEVEL = 5  min precision: 16.17 bits
 * POKI_ATAN_LEVEL = 6  min precision: 18.95 bits
 * POKI_ATAN_LEVEL = 7  min precision: 21.30 bits
 */
#define POKI_ATAN_LEVEL 7
#endif

#ifndef POKI_ASIN_ACOS_LEVEL
/*
 * PRECISION FOR ASIN / ACOS COMPUTATIONS
 * POKI_ASIN_ACOS_LEVEL = 3  min precision: 13.10 bits
 * POKI_ASIN_ACOS_LEVEL = 4  min precision: 15.99 bits
 * POKI_ASIN_ACOS_LEVEL = 5  min precision: 18.78 bits
 * POKI_ASIN_ACOS_LEVEL = 6  min precision: 19.97 bits
 * POKI_ASIN_ACOS_LEVEL = 7  min precision: 19.97 bits
 */
#define POKI_ASIN_ACOS_LEVEL 7
#endif


/***
 * Public function prototypes
 ***/
 //direct trigonometric functions
float poki_sin_f32(float x);
float poki_cos_f32(float x);
float poki_tan_f32(float x);
float poki_cot_f32(float x);

//compute both sine and cosine, optimize for speed
void poki_sincos_f32(float * ysin, float * ycos, float x);

//inverse trigonometric functions
float poki_acos_f32(float x);
float poki_asin_f32(float x);
float poki_atan_f32(float x);
float poki_atan2_f32(float y, float x);


#endif
