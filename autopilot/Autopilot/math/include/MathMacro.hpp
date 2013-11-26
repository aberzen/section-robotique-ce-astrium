/*
 * MathMacro.hpp
 *
 *  Created on: 11 janv. 2013
 *      Author: Aberzen
 */

#ifndef MATHMACRO_HPP_
#define MATHMACRO_HPP_

#include <math.h>

/** @brief Convert degree to radiant */
#define DEG_TO_RAD (M_PI/180.)

/** @brief Convert radiant to degree*/
#define RAD_TO_DEG (180./M_PI)

/** @brief Minimum among two values */
#define math_min(a, b) \
	(((a)<(b)) ? (a) : (b))

/** @brief Maximum among two values */
#define math_max(a, b) \
	(((a)>(b)) ? (a) : (b))

/** @brief Absolute value */
#define math_abs(a) \
	(((a)<0) ? (-(a)) : (a))

/** @brief Sign */
#define math_sign(a) \
	(((a)<0) ? (-1) : (1))

#endif /* MATHMACRO_HPP_ */
