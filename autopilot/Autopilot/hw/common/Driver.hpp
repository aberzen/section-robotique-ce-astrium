/*
 * Driver.h
 *
 *  Created on: 7 janv. 2013
 *      Author: Aberzen
 */

#ifndef MYDRIVER_H_
#define MYDRIVER_H_

namespace hw {


class Driver
{
public:
	Driver();
	virtual ~Driver();

	/** @brief Initialize the HW */
	virtual bool initialize() ;

	/** @brief Initialize the HW */
	inline bool isInitialized() const ;

	/** @brief Reset the HW */
	virtual void reset();

protected:
	bool _isInitialized;
};

bool Driver::isInitialized() const
{
	return Driver::_isInitialized;
}

} /* namespace hw */
#endif /* DRIVER_H_ */
