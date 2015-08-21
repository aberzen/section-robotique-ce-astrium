/*
 * Gps.hpp
 *
 *  Created on: 24 déc. 2013
 *      Author: Robotique
 */

#ifndef GPS_HPP_
#define GPS_HPP_

#include <hw/common/Driver.hpp>
#include <math/Vector3.hpp>
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
	virtual bool initialize();

	/** @brief Execute the process */
	virtual void execute();

	/** @brief Reset the HW */
	virtual void reset();


protected:
	/* Outputs */
	Output& _out;
};

} /* namespace hw */

#endif /* GPS_HPP_ */
