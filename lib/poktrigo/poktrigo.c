#include <stdint.h>
#include <math.h>
#include "poktrigo.h"

static const float _poki_const_pi = 3.141592653589793f;
static const float _poki_const_2_pi = (2.0f / _poki_const_pi);
static const float _poki_const_pi_2 = (_poki_const_pi / 2.0f);
static const float _poki_const_pi_4 = (_poki_const_pi / 4.0f);

// Private functions
// Interpolation
__attribute__((always_inline)) static inline float _poki_sin_poly(float x);
__attribute__((always_inline)) static inline float _poki_atan_poly(float x);
__attribute__((always_inline)) static inline float _poki_tan_poly(float x);
__attribute__((always_inline)) static inline float _poki_acos_poly(float x);
// angle normalization
__attribute__((always_inline)) static inline float _poki_normalize_sin(float x);
__attribute__((always_inline)) static inline float _poki_normalize_tan(float x);


/******************************************************************************
 * Public Functions
 ******************************************************************************/

//direct trigonometric functions
__attribute__((flatten))
	float poki_sin_f32(float x){
		//all computation are done on -1..1 -> [-Pi/2 .. Pi/2]
		x = x * _poki_const_2_pi;		
		//get fractional part of angle
		float xf =  _poki_normalize_sin(x); 
		return _poki_sin_poly(xf);
	}

__attribute__((flatten))
	float poki_cos_f32(float x){
		//all computation are done on -1..1 -> [-Pi/2 .. Pi/2]
		//add 1.0f (Pi/2) to compute cosine with sine polynom 
		x = x * _poki_const_2_pi + 1.0f;
		//use sine polynom
		float xf =  _poki_normalize_sin(x);
		return _poki_sin_poly(xf);
	}

__attribute__((flatten))
	float poki_tan_f32(float x){
		//all computation are done on -1..1 -> [-Pi/2 .. Pi/2]
		float xf = x * _poki_const_2_pi;
		float xn =_poki_normalize_tan(xf);
		return _poki_tan_poly(xn);
	}

__attribute__((flatten))
	float poki_cot_f32(float x){
		//uses tan function to compute cotangent
		float xf = x * _poki_const_2_pi;
		float xn =_poki_normalize_tan(1.0f - xf);
		return _poki_tan_poly(xn);
	}

__attribute__((flatten))
	void poki_sincos_f32(float * ysin, float * ycos, float x){
		x = x * _poki_const_2_pi; 	
		int32_t xi = (int32_t) x;
		// bit trick: xe is the first odd integer less or equal to x
		float xo = (float)(xi | 0x01);

		//this part basically transforms the input (x) to a symmetrical sawtooth
		//function oscillating between -1 and 1, of period 4. 
		//Since [0, 1] -> [0, Pi/2], we map the input to [0, Pi]
		float xsin = fabsf(x - xo);
		//Add 1 (Pi/2) to get cosine, since sin(x + Pi/2) = cos(x)
		float xcos = fabsf(x - xo + 1.0f);
		//change slope between 1 and -1 when integer part changes 
		if (xi & 0x02){
			xsin = xsin - 1.0;
			xcos = xcos - 1.0f;
		}
		else {
			xsin = 1.0f - xsin;
			xcos = 1.0f - xcos;
		}

		//apply interpolation polynom.
		*ysin = _poki_sin_poly(xsin);
		*ycos = _poki_sin_poly(xcos);

	}

//inverse trigonometric functions
__attribute__((flatten))
	float poki_acos_f32(float x){
		float xa = fabsf(x);
		float xn = sqrtf(1.0f - xa);
		float y = _poki_acos_poly(xn);
		if (x < 0.0f){
			y = _poki_const_pi - y;
		}
		return y;
	}

__attribute__((flatten))
	float poki_asin_f32(float x){
		//use trigonometric relation between arcsine and arccosine
		return _poki_const_pi_2 - poki_acos_f32(x);
	}

__attribute__((flatten))
	float poki_atan_f32(float x){
		float xa = fabsf(x);
		float xn = (xa - 1.0f) / (xa + 1.0f);
		float y = _poki_atan_poly(xn) + _poki_const_pi_4;
		return x >= 0.0f ? y : -y;
	}

__attribute__((flatten)) 
	float poki_atan2_f32(float y, float x){
		// get sign of x, y
		float sx = x >= 0.0f ? 1.0f : -1.0f;
		float sy = y >= 0.0f ? 1.0f : -1.0f;

		//get reduced variable (to approximate with atan polynom)
		float xa = fabs(x);
		float ya = fabs(y);
		float xatan = (ya - xa) / (ya + xa);

		//get polynom
		float z = _poki_atan_poly(xatan);
		z = z - _poki_const_pi_4;

		//negate and offset by Pi/2 if needed (depending of sign of x and y)
		return sx * sy * z + sy * _poki_const_pi_2;
	}

/******************************************************************************
 * Private Functions
 ******************************************************************************/

/*
 * sine / cosine angle normalization
 * maps IR to a [-1, 1] triangular wave
 */
__attribute__((always_inline)) 
	static inline float _poki_normalize_sin(float x) {
		// GNU C or ARM will rounf toward zero
		int32_t xi = (int) x;
		//bit trick, get the nearest odd integer
		float xo = (float)(xi | 0x01);

		//transform a -1..1 curve to 0..1
		float xf = fabsf(x - xo);
		//change slope if needed (triangulat wave)
		if (xi & 0x02){
			xf = xf - 1.0f;
		}
		else{
			xf = 1.0f - xf;
		}
		return xf;
	}

/*
 * tan angle normalization
 * maps IR to a [0, 1] sawtooth wave
 */
__attribute__((always_inline)) 
	static inline float _poki_normalize_tan(float x) {
		int32_t xi = (int32_t)x;

		//round toward -Inf instead of toward zero
		if (x < 0.0f){
			xi = (xi - 1) | 0x01;
		}
		else{
			xi = xi | 0x01;
		}

		return x - (float)xi;
	}

/*
 * Sine interpolation polynom
 * Computed for a flat error over definition range
 * Works for x in [-1, 1] (maps to [-Pi/2, Pi/2])
 */
__attribute__((always_inline)) static inline 
float _poki_sin_poly(float x){
	float y = 0.0f;
	float x2 = x * x;
#if POKI_SIN_COS_LEVEL == 3
	y = 7.1451309125e-02f;
	y = -6.4168783552e-01f + y * x2;
	y = 1.5702365264e+00f  + y * x2;
#elif POKI_SIN_COS_LEVEL == 4
	y = -4.3221884008e-03f;
	y = 7.9417617716e-02f  + y * x2;
	y = -6.4588570449e-01f + y * x2;
	y = 1.5707902752e+00f  + y * x2;
#elif POKI_SIN_COS_LEVEL == 5
	y = 1.5062032368e-04f;
	y = -4.6718215429e-03f + y * x2;
	y = 7.9688209586e-02f  + y * x2;
	y = -6.4596329420e-01f + y * x2;
	y = 1.5707962858e+00f  + y * x2;
#else
#error POKI_SIN_COS_LEVEL not define in correct range, must be between 3 and 5
#endif
	return x * y;
}

/*
 * atan interpolation polynom
 * Computed for a flat error over definition range
 * Works for x in [0, 1]
 */
__attribute__((always_inline)) static inline
float _poki_atan_poly(float x){
	float y = 0.0f;
	float x2 = x * x;
#if POKI_ATAN_LEVEL == 3
	y = 7.4948364883e-02f;
	y = -2.8393679096e-01f + y * x2;
	y = 9.9438658947e-01f  + y * x2;
#elif POKI_ATAN_LEVEL == 4
	y = -3.7154256676e-02f;
	y = 1.4340039769e-01f  + y * x2;
	y = -3.1994021825e-01f + y * x2;
	y = 9.9909224063e-01f  + y * x2;
#elif POKI_ATAN_LEVEL == 5
	y = 2.0034121321e-02f;
	y = -8.3508800725e-02f + y * x2;
	y = 1.7907406786e-01f  + y * x2;
	y = -3.3005433849e-01f + y * x2;
	y = 9.9985311343e-01f  + y * x2;
#elif POKI_ATAN_LEVEL == 6
	y = -1.1354850764e-02f;
	y = 5.1743326350e-02f  + y * x2;
	y = -1.1563168253e-01f + y * x2;
	y = 1.9324863538e-01f  + y * x2;
	y = -3.3258349314e-01f + y * x2;
	y = 9.9997622810e-01f  + y * x2;
#elif POKI_ATAN_LEVEL == 7
	y = 6.6519572832e-03f;
	y = -3.3140975622e-02f + y * x2;
	y = 7.9122316010e-02f  + y * x2;
	y = -1.3208586487e-01f + y * x2;
	y = 1.9802463253e-01f  + y * x2;
	y = -3.3317005492e-01f + y * x2;
	y = 9.9999615300e-01f  + y * x2;
#else
#error POKI_ATAN_LEVEL not define in correct range, must be between 3 and 7
#endif
	return x * y;
}

/*
 * tan interpolation polynom
 * Computed for a flat error over definition range
 * Works for x in [0, 1] (maps to [0, Pi/2])
 */
__attribute__((always_inline)) static inline
float _poki_tan_poly(float x){
	float y = 0.0f;
	float x2 = x * x;
#if POKI_TAN_LEVEL == 3
	y = 1.3649608000e-01f;
	y = 4.9586661000e-01f + y * x2;
	y = -6.3304246000e-01f / x + (y * x);
#elif POKI_TAN_LEVEL == 4
	y = 3.7526650000e-02f;
	y = 7.1492420000e-02f + y * x2;
	y = 5.2787942000e-01f + y * x2;
	y = -6.3694694000e-01f / x + (y * x);
#elif POKI_TAN_LEVEL == 5
	y = 1.0687760000e-02f;
	y = 1.3670690000e-02f + y * x2;
	y = 8.9196330000e-02f + y * x2;
	y = 5.2303286000e-01f + y * x2;
	y = -6.3659112000e-01f / x + (y * x);
#elif POKI_TAN_LEVEL == 6
	y = 3.0639900000e-03f;
	y = 2.3167600000e-03f + y * x2;
	y = 2.1973950000e-02f + y * x2;
	y = 8.5601030000e-02f + y * x2;
	y = 5.2366621000e-01f + y * x2;
	y = -6.3662220000e-01f / x + (y * x);
#elif POKI_TAN_LEVEL == 7
	y = 8.7959914500e-04f;
	y = 2.2107970600e-04f + y * x2;
	y = 5.8470051400e-03f + y * x2;
	y = 1.9872711400e-02f + y * x2;
	y = 8.6207847300e-02f + y * x2;
	y = 5.2359131000e-01f + y * x2;
	y = -6.3661957100e-01f / x + (y * x);
#else
#error POKI_TAN_LEVEL not define in correct range, must be between 3 and 7
#endif
	return y;
}

/*
 * arcosine interpolation polynom
 * Computed for a flat error over definition range
 * Works for x in [0, 1]
 */
__attribute__((always_inline)) static inline
float _poki_acos_poly(float x){
	float y = 0.0f;
#if POKI_ASIN_ACOS_LEVEL == 3
	y = -2.3325400000e-02f;
	y = 8.0623410000e-02f  + y * x;
	y = -2.1415105000e-01f + y * x;
	y = 1.5707963300e+00f  + y * x;
#elif POKI_ASIN_ACOS_LEVEL == 4
	y = 1.0960870000e-02f;
	y = -4.0114590000e-02f + y * x;
	y = 8.7148630000e-02f  + y * x;
	y = -2.1454379000e-01f + y * x;
	y = 1.5707963300e+00f  + y * x;
#elif POKI_ASIN_ACOS_LEVEL == 5
	y = -5.5641500000e-03f;
	y = 2.2274930000e-02f  + y * x;
	y = -4.7354850000e-02f + y * x;
	y = 8.8650820000e-02f  + y * x;
	y = -2.1459400000e-01f + y * x;
	y = 1.5707963300e+00f  + y * x;
#elif POKI_ASIN_ACOS_LEVEL == 6
	y = 2.9721100000e-03f;
	y = -1.3097310000e-02f + y * x;
	y = 2.8985830000e-02f  + y * x;
	y = -4.9811020000e-02f + y * x;
	y = 8.8968980000e-02f  + y * x;
	y = -2.1460074000e-01f + y * x;
	y = 1.5707963300e+00f  + y * x;
#elif POKI_ASIN_ACOS_LEVEL == 7
	y = -1.6459800000e-03f;
	y = 7.9685700000e-03f  + y * x;
	y = -1.8800930000e-02f + y * x;
	y = 3.1998650000e-02f  + y * x;
	y = -5.0534630000e-02f + y * x;
	y = 8.9033140000e-02f  + y * x;
	y = -2.1460168000e-01f + y * x;
	y = 1.5707963300e+00f  + y * x;
#else
#error POKI_ASIN_ACOS_LEVEL not define in correct range, must be between 3 and 7
#endif
	return y;
}

