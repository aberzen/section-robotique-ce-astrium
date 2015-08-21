/*
 * OutputStream.hpp
 *
 *  Created on: 6 mai 2015
 *      Author: Aberzen
 */

#ifndef OUTPUTSTREAM_HPP_
#define OUTPUTSTREAM_HPP_

#include <inttypes.h>
#include <stddef.h>

namespace hw {

class OutputStream {
public:
	OutputStream();
	virtual ~OutputStream();

	/** Closes this output stream and releases any system resources associated with this stream. */
	virtual void close() = 0;

	/** Flushes this output stream and forces any buffered output bytes to be written out. */
	virtual void flush() = 0;

	/** Writes b.length bytes from the specified byte array to this output stream. */
	virtual size_t write(const uint8_t *b, size_t len) = 0;

	/** Writes len bytes from the specified byte array starting at offset off to this output stream. */
	virtual size_t write(const uint8_t *b, size_t off, size_t len) = 0;

	/** Writes the specified byte to this output stream. */
	virtual size_t write(uint8_t b) = 0;

	/** Get Tx freespace. */
	virtual size_t freespace() = 0;
};

} /* namespace hw */
#endif /* OUTPUTSTREAM_HPP_ */
