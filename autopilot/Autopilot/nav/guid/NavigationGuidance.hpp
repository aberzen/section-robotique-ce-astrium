/*
 * NavigationGuidance.hpp
 *
 *  Created on: 21 août 2015
 *      Author: AdministrateurLocal
 */

#ifndef NAVIGATION_GUID_NAVIGATIONGUIDANCE_HPP_
#define NAVIGATION_GUID_NAVIGATIONGUIDANCE_HPP_

#include <math/Vector3.hpp>

#define NAVIGATION_GUIDANCE_IDX_X   (0)
#define NAVIGATION_GUIDANCE_IDX_Y   (1)
#define NAVIGATION_GUIDANCE_IDX_Z   (2)

namespace navigation {

class NavigationGuidance {
public:
	NavigationGuidance();
	virtual ~NavigationGuidance();

	/** @brief Initialize internal state */
	virtual void initialize(
			const math::Vector3f& position_I,
			const math::Vector3f& velocity_I) = 0;

	void execute();
};

} /* namespace navigation */

#endif /* NAVIGATION_GUID_NAVIGATIONGUIDANCE_HPP_ */
