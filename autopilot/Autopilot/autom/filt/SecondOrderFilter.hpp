/*
 * SecondOrderFilter.hpp
 *
 *  Created on: 12 oct. 2015
 *      Author: AdministrateurLocal
 */

#ifndef AUTOM_FILT_SECONDORDERFILTER_HPP_
#define AUTOM_FILT_SECONDORDERFILTER_HPP_

#include "Filter.hpp"

namespace autom {

/** @brief Implement optimized second order filter
 *
 * This is the Oppenheim Schafer form of a second order filter
 *
 * Implement the filter:
 *
 * Y(z)   a0 + a1*(1/z) + a2*(1/z)^2
 * ---- = --------------------------
 * X(z)    1 - b1*(1/z) - b2*(1/z)^2
 *
 * which can be rewritten as:
 * Y(t) = a0*X(t)+a1*X(t-T)+a2*X(t-2T)+b1*Y(t-T)+b2*Y(t-2T)
 *
 * An optimized implementation allows removing one state:
 * Y(t) = U(t) + a0*X(t) + a1*X(t-T) + b1*Y(t-T)
 * U(t+T) = a2*X(t-T) + b2*Y(t-T)
 *
 */

class SecondOrderFilter : public Filter<float> {
public:
	typedef struct
	{
		float a0;
		float a1;
		float a2;
		float b1;
		float b2;
	} Parameter;

public:
	SecondOrderFilter(const Parameter& param);
	virtual ~SecondOrderFilter();

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual float        apply(float sample);

    // reset - clear the filter
    virtual void        reset(float value);

protected:
    /** @brief X(n-T) */
    float _Xprev;
    /** @brief Y(n-T) */
    float _Yprev;
    /** @brief U(n+T) */
    float _Unext;
    /** @brief Parameter */
    const Parameter& _param;
};

} /* namespace autom */

#endif /* AUTOM_FILT_SECONDORDERFILTER_HPP_ */
