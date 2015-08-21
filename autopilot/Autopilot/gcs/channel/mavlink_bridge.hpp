/*
 * mavlink_config.hpp
 *
 *  Created on: 1 févr. 2015
 *      Author: Aberzen
 */

#ifndef MAVLINK_BRIDGE_HPP_
#define MAVLINK_BRIDGE_HPP_

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
#include <mavlink_types.h>

/* Struct that stores the communication settings of this system.
   you can also define / alter these settings elsewhere, as long
   as they're included BEFORE mavlink.h.
   So you can set the

   mavlink_system.sysid = 100; // System ID, 1-255
   mavlink_system.compid = 50; // Component/Subsystem ID, 1-255

   Lines also in your main.c, e.g. by reading these parameter from EEPROM.
 */
extern mavlink_system_t mavlink_system;

/**
 * @brief Send one char (uint8_t) over a comm channel
 *
 * @param chan MAVLink channel to use, usually MAVLINK_COMM_0 = UART0
 * @param ch Character to send
 */
extern void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
extern void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t* buf, uint16_t len);

#include <ardupilotmega/mavlink.h>
#endif /* MAVLINK_BRIDGE_HPP_ */
