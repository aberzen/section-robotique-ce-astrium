/*
 * SecondOrderFilter.cpp
 *
 *  Created on: 12 oct. 2015
 *      Author: AdministrateurLocal
 */

#include <autom/filt/SecondOrderFilter.hpp>

namespace autom {

SecondOrderFilter::SecondOrderFilter(const Parameter& param)
: Filter(),
  _Xprev(0.),
  _Yprev(0.),
  _Unext(0.),
  _param(param)
{
}

SecondOrderFilter::~SecondOrderFilter()
{
}

// apply - Add a new raw value to the filter, retrieve the filtered result
float SecondOrderFilter::apply(float sample)
{
	/* Y(t) = U(t) + a0*X(t) + a1*X(t-T) + b1*Y(t-T) */
	float Y = _Unext + _param.a0 * sample + _param.a1 * _Xprev + _param.b1 * _Yprev;

	/* U(t+T) = a2*X(t-T) + b2*Y(t-T) */
	_Unext = _param.a2 * _Xprev + _param.b2 * _Yprev;

	/* Save X and Y */
	_Xprev = sample;
	_Yprev = Y;

	/* Return filtered value */
	return _Yprev;
}

// reset - clear the filter
void SecondOrderFilter::reset(float value)
{
	/* State is such that an infinite time was spent with this entry.
	 * The filter has converged toward this value.
	 *
	 * (1 - b1 - b2)*Y_conv = (a0 + a1 + a2)*X_conv
	 *
	 * Can be rewritten as:
	 *          a0 + a1 + a2
	 * Y_conv = ------------ * X_conv
	 *           1 - b1 - b2
	 */

	_Xprev = value;
	_Yprev = (_param.a0 + _param.a1 + _param.a2) * value / (1 - _param.b1 - _param.b2) ;
	_Unext = _param.a2 * _Xprev + _param.b2 * _Yprev;
}


} /* namespace autom */
