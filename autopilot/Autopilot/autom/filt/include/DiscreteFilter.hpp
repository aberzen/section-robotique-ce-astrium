/*
 * DiscreteFilter.h
 *
 *  Created on: 12 janv. 2013
 *      Author: Aberzen
 */

#ifndef DISCRETEFILTER_H_
#define DISCRETEFILTER_H_

#include "Filter.hpp"

namespace autom {

/** @brief Implement generic discrete filter
 *
 * Implement the filter:
 *
 * Y(z)   a0 + a1*(1/z) + ... + am*(1/z)^m
 * ---- = --------------------------------
 * X(z)   b0 + b1*(1/z) + ... + bn*(1/z)^n
 *
 * which can be rewritten as:
 * Y(t) = [ (a0*X(t)+a1*X(t-T)+...+am*X(t-mT)) - (b1*Y(t-T)+...+bn*Y(t-nT)) ] / b0
 *
 * As a consequence, b0 can not be null
 */

template <class T, class U, uint8_t FILTER_SIZE_X, uint8_t FILTER_SIZE_Y>
class DiscreteFilter : public Filter<T> {
public:
	DiscreteFilter(
			const T coeffX[FILTER_SIZE_Y],
			const T coeffY[FILTER_SIZE_X]
			);
	virtual ~DiscreteFilter();

	// setCoeff - setter method for coefficient
	void setCoeff(const T coeffX[FILTER_SIZE_X], const T coeffY[FILTER_SIZE_Y]);

	// apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T apply(T sample);

    // reset - clear the filter
    virtual void reset();

    // reset to the initial conditions
    virtual void reset(const T initCondX[FILTER_SIZE_X], const T initCondY[FILTER_SIZE_Y]);

protected:
    /** @brief Pointer to the next empty slot in the buffer */
    uint8_t _sampleIdxY;
    /** @brief Pointer to the next empty slot in the buffer */
    uint8_t _sampleIdxX;

    /** @brief bk coefficients of the filter */
    T _coeffY[FILTER_SIZE_Y];
    /** @brief ak coefficients of the filter */
    T _coeffX[FILTER_SIZE_X];

    /** @brief Saved Y(t-kT) values */
    T _valuesY[FILTER_SIZE_Y];
    /** @brief Saved X(t-Kt) values */
    T _valuesX[FILTER_SIZE_X];
};

} /* namespace autom */

/* Implementation and declaration of classes with template must be in the same file */
#include "../source/DiscreteFilter.cpp_"

#endif /* DISCRETEFILTER_H_ */
