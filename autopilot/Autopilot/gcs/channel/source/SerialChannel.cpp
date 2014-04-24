/*
 * SerialChannel.cpp
 *
 *  Created on: 1 juin 2013
 *      Author: Aberzen
 */

#include "../include/SerialChannel.hpp"

namespace mavlink {

SerialChannel::SerialChannel(mavlink_channel_t chan, FastSerial& serial):
	Channel(chan),
	_serial(serial) {
}
SerialChannel::~SerialChannel(){
}

bool SerialChannel::receiveMessage(mavlink_message_t* r_message) {
	while(_serial.available())
	{
		if (this->parseChar((uint8_t)_serial.read(),r_message)) {
			return true;
		}
	}
	return false;
}

void SerialChannel::sendChar(uint8_t ch) {
	_serial.write(ch);
}


} /* namespace mavlink */
