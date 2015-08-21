/*
 * HalImu.hpp
 *
 *  Created on: 6 juil. 2013
 *      Author: Aberzen
 */

#ifndef HALIMU_HPP_
#define HALIMU_HPP_

#include <math/Vector3.hpp>
#include <hw/common/Driver.hpp>

namespace hw {

class HalImu : public hw::Driver {
public:
	HalImu();
	virtual ~HalImu();

	virtual void getGyrOffsets(math::Vector3i& gyrOffsets_U) const = 0;
	virtual void setGyrOffsets(const math::Vector3i& gyrOffsets_U) = 0;

	virtual void getAccOffsets(math::Vector3i& accOffsets_U) const = 0;
	virtual void setAccOffsets(const math::Vector3i& accOffsets_U) = 0;

	virtual bool sample (
			math::Vector3i& gyroMeas_U,
			math::Vector3i& accoMeas_U,
			int16_t& temperature) = 0;
};

} /* namespace hw */
#endif /* HALIMU_HPP_ */
