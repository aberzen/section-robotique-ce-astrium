/*
 * DataStream.hpp
 *
 *  Created on: 22 juil. 2013
 *      Author: Aberzen
 */

#ifndef DATASTREAM_HPP_
#define DATASTREAM_HPP_

#include <common/mavlink.h>
#include <infra/app/include/Process.hpp>

namespace mavlink {

class DataStream : public infra::Process{
public:
	DataStream(mavlink_channel_t port);
	virtual ~DataStream();

	/** @brief Update data stream */
	void update();

	/** @brief Stop streaming */
	void stop();

	/** @brief Start streaming */
	void start(uint16_t periodMs);

	/** @brief Init the process */
	virtual infra::status initialize();

	/** @brief Execute the process */
	virtual infra::status execute();

protected:
	/** @brief Sample the stream */
	void sample();

	/** @brief Sample the data */
	virtual void sampleData() = 0;

	/** @brief Sample the date */
	void sampleDate();

	/** @brief Send data */
	virtual void sendData() = 0;

	/** @brief Increment counter */
	bool increment();

	/** @brief Increment counter */
	void reset();


protected:
	/** @brief Mavlink port to be used */
	mavlink_channel_t _port;

	/** @brief Period (tick) */
	uint16_t _period;

	/** @brief Counter (tick) */
	uint16_t _counter;

	/** @brief Timestamp (microseconds since UNIX epoch or microseconds since system boot) */
	uint64_t _date;

};

} /* namespace mavlink */
#endif /* DATASTREAM_HPP_ */
