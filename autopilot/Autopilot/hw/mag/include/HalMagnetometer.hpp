/*
 * HalMagnetometer.hpp
 *
 *  Created on: 31 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALMAGNETOMETER_HPP_
#define HALMAGNETOMETER_HPP_

#include <hw/common/include/Driver.hpp>
#include <math/include/Vector3.hpp>

namespace hw {

class HalMagnetometer : public Driver {
public:
	typedef struct
	{
		/** @brief Magnetic field direction */
		math::Vector3f magMeas_B;

		/** @brief Magnetic field direction */
		bool isAvailable;

	} Output ;
	typedef struct
	{
		/** @brief Magnetic field direction */
		math::Vector3i magMeas_B;
	} RawOutput ;
public:
	HalMagnetometer(Output& out, RawOutput& rawOut);
	virtual ~HalMagnetometer();

protected:
	RawOutput& _rawOut;
	Output& _out;
};

} /* namespace hw */
#endif /* HALMAGNETOMETER_HPP_ */
