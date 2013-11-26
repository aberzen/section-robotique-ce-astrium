/*
 * MathUtils.hpp
 *
 *  Created on: 11 janv. 2013
 *      Author: Aberzen
 */

#ifndef MATHUTILS_HPP_
#define MATHUTILS_HPP_

#include <math/include/MathMacro.hpp>

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

}

#endif /* MATHUTILS_HPP_ */
