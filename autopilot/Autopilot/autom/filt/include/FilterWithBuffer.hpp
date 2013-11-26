// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	FilterWithBuffer.h
/// @brief	A filter with a buffer.
///         This is implemented separately to the base Filter class to get around
///         restrictions caused by the use of templates which makes different sizes essentially
///         completely different classes

#ifndef FILTERWITHBUFFER_HPP_
#define FILTERWITHBUFFER_HPP_

#include "Filter.hpp"
#include <inttypes.h>

template <class T, uint8_t FILTER_SIZE>
class FilterWithBuffer : public Filter<T>
{
public:
    // constructor
	FilterWithBuffer();

    // constructor
    virtual ~FilterWithBuffer();

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T apply(T sample);

    // reset - clear the filter
    virtual void reset();

protected:
    T               samples[FILTER_SIZE];       // buffer of samples
    uint8_t         sample_index;               // pointer to the next empty slot in the buffer
};

/* Implementation and declaration of classes with template must be in the same file */
#include "../source/FilterWithBuffer.cpp_"

// Typedef for convenience
typedef FilterWithBuffer<int16_t,2> FilterWithBufferInt16_Size2;
typedef FilterWithBuffer<int16_t,3> FilterWithBufferInt16_Size3;
typedef FilterWithBuffer<int16_t,4> FilterWithBufferInt16_Size4;
typedef FilterWithBuffer<int16_t,5> FilterWithBufferInt16_Size5;
typedef FilterWithBuffer<int16_t,6> FilterWithBufferInt16_Size6;
typedef FilterWithBuffer<int16_t,7> FilterWithBufferInt16_Size7;
typedef FilterWithBuffer<uint16_t,2> FilterWithBufferUInt16_Size2;
typedef FilterWithBuffer<uint16_t,3> FilterWithBufferUInt16_Size3;
typedef FilterWithBuffer<uint16_t,4> FilterWithBufferUInt16_Size4;
typedef FilterWithBuffer<uint16_t,5> FilterWithBufferUInt16_Size5;
typedef FilterWithBuffer<uint16_t,6> FilterWithBufferUInt16_Size6;
typedef FilterWithBuffer<uint16_t,7> FilterWithBufferUInt16_Size7;

typedef FilterWithBuffer<int32_t,2> FilterWithBufferInt32_Size2;
typedef FilterWithBuffer<int32_t,3> FilterWithBufferInt32_Size3;
typedef FilterWithBuffer<int32_t,4> FilterWithBufferInt32_Size4;
typedef FilterWithBuffer<int32_t,5> FilterWithBufferInt32_Size5;
typedef FilterWithBuffer<int32_t,6> FilterWithBufferInt32_Size6;
typedef FilterWithBuffer<int32_t,7> FilterWithBufferInt32_Size7;
typedef FilterWithBuffer<uint32_t,2> FilterWithBufferUInt32_Size2;
typedef FilterWithBuffer<uint32_t,3> FilterWithBufferUInt32_Size3;
typedef FilterWithBuffer<uint32_t,4> FilterWithBufferUInt32_Size4;
typedef FilterWithBuffer<uint32_t,5> FilterWithBufferUInt32_Size5;
typedef FilterWithBuffer<uint32_t,6> FilterWithBufferUInt32_Size6;
typedef FilterWithBuffer<uint32_t,7> FilterWithBufferUInt32_Size7;

#endif



