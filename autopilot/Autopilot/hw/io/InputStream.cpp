/*
 * InputStream.cpp
 *
 *  Created on: 6 mai 2015
 *      Author: Aberzen
 */

#include "InputStream.hpp"

namespace hw {

InputStream::InputStream() {
}

InputStream::~InputStream() {
}

/** Marks the current position in this input stream. */
void InputStream::mark(size_t readlimit) {

}

/** Tests if this input stream supports the mark and reset methods. */
bool InputStream::markSupported() {
	return false;
}

/** Repositions this stream to the position at the time the mark method was last called on this input stream. */
void InputStream::reset() {
}

} /* namespace hw */
