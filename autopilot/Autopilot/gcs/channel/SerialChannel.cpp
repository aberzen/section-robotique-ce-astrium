/*
 * SerialChannel.cpp
 *
 *  Created on: 1 juin 2013
 *      Author: Aberzen
 */

#include "SerialChannel.hpp"
#include <mavlink_helpers.h>

namespace mavlink {

SerialChannel::SerialChannel(mavlink_channel_t chan, hw::Serial& serial)
: Channel(chan),
  _serial(serial)
{
}

SerialChannel::~SerialChannel(){
}

bool SerialChannel::receiveMessage(mavlink_message_t** r_message) {
	*r_message = NULL;
	while(_serial.available())
	{
		if (this->parseChar((uint8_t)_serial.read()))
		{
			*r_message = &_r_message;
			return true;
		}
	}
	return false;
}

bool SerialChannel::sendMessage(const uint8_t* buffer, size_t len)
{
	return len == _serial.write(buffer, len);
}

bool SerialChannel::canSend(size_t len)
{
	return (len<=_serial.freespace());
}

} /* namespace mavlink */
