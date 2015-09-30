/*
 * MathUtils.hpp
 *
 *  Created on: 11 janv. 2013
 *      Author: Aberzen
 */

#ifndef MATHUTILS_HPP_
#define MATHUTILS_HPP_

#include <stdint.h>
#include <math/MathMacro.hpp>

namespace math {

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
extern float safe_asin(float v);

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
extern float safe_sqrt(float v);

extern float iter_invSqrt(float guess, int8_t nIter, const float& S);
}

#endif /* MATHUTILS_HPP_ */
