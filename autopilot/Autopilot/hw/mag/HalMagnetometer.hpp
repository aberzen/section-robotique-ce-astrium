/*
 * HalMagnetometer.hpp
 *
 *  Created on: 31 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALMAGNETOMETER_HPP_
#define HALMAGNETOMETER_HPP_

#include <hw/common/Driver.hpp>
#include <math/Vector3.hpp>

namespace hw {

class HalMagnetometer : public Driver {
public:
	HalMagnetometer();
	virtual ~HalMagnetometer();

	virtual bool sample (
			math::Vector3i& compassMeas_U) = 0;

};

} /* namespace hw */
#endif /* HALMAGNETOMETER_HPP_ */
