/*
 * Driver.h
 *
 *  Created on: 7 janv. 2013
 *      Author: Aberzen
 */

#ifndef DRIVER_H_
#define DRIVER_H_

#include <infra/include/Types.hpp>
#include <arch/include/Process.hpp>

namespace hw {

class Driver : public arch::Process {
public:
	Driver();
	virtual ~Driver();

	/** @brief Initialize the HW */
	virtual status initialize() = 0;

	/** @brief Reset the HW */
	virtual status reset() = 0;

	/** @brief Execute the driver */
	virtual status execute() = 0;
};


} /* namespace hw */
#endif /* DRIVER_H_ */
