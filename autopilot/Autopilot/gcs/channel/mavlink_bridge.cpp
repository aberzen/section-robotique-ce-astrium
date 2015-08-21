/*
 * mavlink_bridge.cpp
 *
 *  Created on: 3 févr. 2015
 *      Author: Aberzen
 */
#include <stddef.h>
#include "mavlink_bridge.hpp"
#include "Channel.hpp"

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t* buf, uint16_t len)
{
	mavlink::Channel* channel = mavlink::Channel::getChannel(chan);
	if (channel != NULL)
	{
		channel->sendMessage(buf, len);
	}
}
