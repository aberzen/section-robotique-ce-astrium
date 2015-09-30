/*
 * NavigationDirectThrust.hpp
 *
 *  Created on: 31 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef NAVIGATION_NAVIGATIONDIRECTTHRUST_HPP_
#define NAVIGATION_NAVIGATIONDIRECTTHRUST_HPP_

#include <hw/radio/Radio.hpp>

namespace navigation {

class NavigationDirectThrust {
public:
	typedef struct {
		int32_t unitThrust[3];
	} Parameter;

public:
	NavigationDirectThrust(const Parameter& param);
	virtual ~NavigationDirectThrust();

	/** @brief Compute guidance */
	virtual void calcGuidance(
			hw::Radio& radio);

protected:
	/** @brief Parameter */
	Parameter _param;

};

} /* namespace navigation */

#endif /* NAVIGATION_NAVIGATIONDIRECTTHRUST_HPP_ */
