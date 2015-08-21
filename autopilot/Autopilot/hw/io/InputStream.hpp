/*
 * InputStream.hpp
 *
 *  Created on: 6 mai 2015
 *      Author: Aberzen
 */

#ifndef INPUTSTREAM_HPP_
#define INPUTSTREAM_HPP_

#include <inttypes.h>
#include <stddef.h>

namespace hw {

class InputStream {
public:
	InputStream();
	virtual ~InputStream();

	/** Returns an estimate of the number of bytes that can be read (or skipped over)
	 * from this input stream without blocking by the next invocation of a method for this input stream. */
	virtual size_t available() = 0;

	/** Closes this input stream and releases any system resources associated with the
	 * stream. */
	virtual void close() = 0;

	/** Marks the current position in this input stream. */
	virtual void mark(size_t readlimit);

	/** Tests if this input stream supports the mark and reset methods. */
	virtual bool markSupported();

	/** Reads the next byte of data from the input stream (0xFFFF for none). */
	virtual uint16_t read() = 0;

	/** Reads some number of bytes from the input stream and stores them into the buffer array b. */
	virtual size_t read(uint8_t *b, size_t len) = 0;

	/** Reads up to len bytes of data from the input stream into an array of bytes. */
	virtual size_t read(uint8_t *b, size_t off, size_t len) = 0;

	/** Repositions this stream to the position at the time the mark method was last called on this input stream. */
	virtual void reset();

	/** Skips over and discards n bytes of data from this input stream. */
	virtual size_t skip(size_t n) = 0;
};

} /* namespace hw */
#endif /* INPUTSTREAM_HPP_ */
