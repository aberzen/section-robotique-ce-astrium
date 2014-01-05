/*
 * Driver.h
 *
 *  Created on: 7 janv. 2013
 *      Author: Aberzen
 */

#ifndef MYDRIVER_H_
#define MYDRIVER_H_

#include <arch/app/include/Process.hpp>
namespace hw {


class Driver : public infra::Process
{
public:
	Driver();
	virtual ~Driver();

	/** @brief Reset the HW */
	virtual infra::status reset() = 0;
};


} /* namespace hw */
#endif /* DRIVER_H_ */
