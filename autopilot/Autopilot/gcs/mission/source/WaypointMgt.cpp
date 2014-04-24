/*
 * WaypointMgt.cpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#include "../include/WaypointMgt.hpp"

namespace mavlink {

WaypointMgt::WaypointMgt() {
}

WaypointMgt::~WaypointMgt() {

}

/** @brief Clear all waypoints */
void WaypointMgt::clearAll()
{

}



/** @brief Clear all waypoints */
void WaypointMgt::ack(
		uint8_t type
)
{

}



/** @brief Request waypoints list */
void WaypointMgt::requestList()
{

}



/** @brief Request waypoints list */
void WaypointMgt::request(uint8_t seq)
{

}



/** @brief Select current waypoint */
void WaypointMgt::setCurrent(
		uint16_t seq
)
{

}



/** @brief Set waypoints count */
void WaypointMgt::setCount(
		uint16_t count
)
{

}



/** @brief Set a waypoint */
void WaypointMgt::setItem(
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
)
{

}




} /* namespace mavlink */
