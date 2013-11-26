/*
 * Channel.hpp
 *
 *  Created on: 1 juin 2013
 *      Author: Aberzen
 */

#ifndef CHANNEL_HPP_
#define CHANNEL_HPP_

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include <mavlink/v1.0/mavlink_types.h>
extern void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
extern mavlink_system_t mavlink_system;
#include <mavlink/v1.0/ardupilotmega/mavlink.h>

namespace mavlink {

class Channel {
	friend void ::comm_send_ch(mavlink_channel_t chan, uint8_t ch);
public:
	static Channel* getChannel(mavlink_channel_t chan);

protected:
	static Channel* _channels[MAVLINK_COMM_3+1];
	static void setChannel(mavlink_channel_t chan, Channel* channel);

public:
	Channel(mavlink_channel_t chan);
	virtual ~Channel();

	virtual bool receiveMessage(mavlink_message_t* r_message) = 0;
	mavlink_status_t* getStatus();

protected:
	mavlink_channel_t _chan;

	uint8_t parseChar(uint8_t c, mavlink_message_t* r_message);
	virtual void sendChar(uint8_t ch) = 0;

};

} /* namespace mavlink */
#endif /* CHANNEL_HPP_ */
