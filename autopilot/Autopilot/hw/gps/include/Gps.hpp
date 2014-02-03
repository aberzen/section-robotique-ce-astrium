/*
 * Gps.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef GPS_HPP_
#define GPS_HPP_

#include <hw/common/include/Driver.hpp>
#include <math/include/Vector3.hpp>
#include "GpsPosition.hpp"

namespace hw {

class Gps : public Driver {
public:
	typedef struct {
		GpsPosition posMeas_I;
		::math::Vector3f velMeas_I;
		bool isAvailable;
	} Output;
	typedef struct {
	} RawOutput;

public:
	Gps(
			/* Outputs */
			Output& out
			);
	virtual ~Gps();

	/** @brief Init the process */
	virtual infra::status initialize();

	/** @brief Execute the process */
	virtual infra::status execute();

	/** @brief Reset the HW */
	virtual infra::status reset();


protected:
	/* Outputs */
	Output& _out;
};

} /* namespace hw */

#endif /* GPS_HPP_ */
