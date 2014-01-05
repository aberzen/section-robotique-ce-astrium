/*
 * Channel.cpp
 *
 *  Created on: 1 juin 2013
 *      Author: Aberzen
 */

#include <gcs/include/Channel.hpp>
#include <protocol.h>

namespace mavlink {

Channel* Channel::_channels[MAVLINK_COMM_3+1] = {NULL, NULL, NULL, NULL};

Channel* Channel::getChannel(mavlink_channel_t chan) {
	if (0<=chan && chan<=MAVLINK_COMM_3){
		return _channels[chan];
	}
	return NULL;

}


void Channel::setChannel(mavlink_channel_t chan, Channel* channel) {
	if (0<=chan && chan<=MAVLINK_COMM_3){
		_channels[chan] = channel;
	}

}

Channel::Channel(mavlink_channel_t chan) :
		_chan(chan) {
	Channel::setChannel(_chan, this);
}

Channel::~Channel() {
}

mavlink_status_t* Channel::getStatus() {
	return mavlink_get_channel_status(_chan);
}

uint8_t Channel::parseChar(uint8_t c, mavlink_message_t* r_message){
	mavlink_status_t r_mavlink_status;
	return mavlink_parse_char(this->_chan, c, r_message, &r_mavlink_status);
}



} /* namespace mavlink */


void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	mavlink::Channel* channel = mavlink::Channel::getChannel(chan);
	if (channel != NULL)
		channel->sendChar(ch);
}
