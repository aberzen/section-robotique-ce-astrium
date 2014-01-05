/*
 * MathUtils.cpp
 *
 *  Created on: 11 janv. 2013
 *      Author: Aberzen
 */

#include "../include/MathUtils.hpp"
#include <math.h>

namespace math {

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0) {
        return M_PI/2;
    }
    if (v <= -1.0) {
        return -M_PI/2;
    }
    return asin(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrt(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}


float iter_invSqrt(float guess, int8_t nIter, const float& S)
{
	while(nIter--)
	{
		guess = guess * 0.5 * (3.- S * guess * guess);
	}
	return guess;
}

}
