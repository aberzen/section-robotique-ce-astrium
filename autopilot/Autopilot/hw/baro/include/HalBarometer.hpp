/*
 * HalBarometer.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALBAROMETER_HPP_
#define HALBAROMETER_HPP_

#include <hw/common/include/Driver.hpp>

namespace hw {

class HalBarometer : public Driver {
public:
	typedef struct {
		/** @brief Pressure in mBar */
		float pressure;
		/** @brief Temperature in cDeg  */
		int32_t temperature;
		/** @brief Are measurement available */
		bool isAvailable;
	} Output;
	typedef struct {
	} RawOutput;

public:
	HalBarometer(
			/* Inputs */
			/* Outputs */
			Output& out
			/* Parameters */
			);
	virtual ~HalBarometer();

	/** @brief Initialize the HW */
	virtual void initialize() ;

	/** @brief Reset the HW */
	virtual void reset() ;

	/** @brief Execute the driver */
	virtual void execute() = 0;

protected:
	/** @brief Output interface*/
	Output& _out;
};

} /* namespace hw */
#endif /* HALBAROMETER_HPP_ */
