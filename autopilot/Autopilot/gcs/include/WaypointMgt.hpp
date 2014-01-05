/*
 * WaypointMgt.hpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#ifndef WAYPOINTMGT_HPP_
#define WAYPOINTMGT_HPP_

#include <stdint.h>

namespace mavlink {

class WaypointMgt {
public:
	WaypointMgt();
	virtual ~WaypointMgt();

	/** @brief Clear all waypoints */
	void clearAll();

	/** @brief Clear all waypoints */
	void ack(
			uint8_t type
	);

	/** @brief Request waypoints list */
	void requestList();

	/** @brief Request waypoints list */
	void request(uint8_t seq);

	/** @brief Select current waypoint */
	void setCurrent(
			uint16_t seq
	);

	/** @brief Set waypoints count */
	void setCount(
			uint16_t count
	);

	/** @brief Set a waypoint */
	void setItem(
			float param1,
			float param2,
			float param3,
			float param4,
			float x,
			float y,
			float z,
			uint16_t seq,
			uint16_t command,
			uint8_t frame,
			uint8_t current,
			uint8_t autocontinue
	);

};

} /* namespace mavlink */
#endif /* WAYPOINTMGT_HPP_ */
