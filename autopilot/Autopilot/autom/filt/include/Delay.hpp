// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	Delay.h
/// @brief	A filter with a buffer.
///         This is implemented separately to the base Filter class to get around
///         restrictions caused by the use of templates which makes different sizes essentially
///         completely different classes

#ifndef DELAY_HPP_
#define DELAY_HPP_

#include <inttypes.h>
#include <FilterWithBuffer.hpp>

template <class T, uint8_t DELAY_STEP>
class Delay : public FilterWithBuffer<T,DELAY_STEP>
{
public:
    // constructor
	Delay();

    // constructor
    virtual ~Delay();

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T apply(T sample);

protected:
    T               samples[DELAY_STEP];       // buffer of samples
    uint8_t         sample_index;               // pointer to the next empty slot in the buffer
};

/* Implementation and declaration of classes with template must be in the same file */
#include "../source/Delay.cpp_"

// Typedef for convenience
typedef Delay<int16_t,2> DelayInt16_Size2;
typedef Delay<int16_t,3> DelayInt16_Size3;
typedef Delay<int16_t,4> DelayInt16_Size4;
typedef Delay<int16_t,5> DelayInt16_Size5;
typedef Delay<int16_t,6> DelayInt16_Size6;
typedef Delay<int16_t,7> DelayInt16_Size7;
typedef Delay<uint16_t,2> DelayUInt16_Size2;
typedef Delay<uint16_t,3> DelayUInt16_Size3;
typedef Delay<uint16_t,4> DelayUInt16_Size4;
typedef Delay<uint16_t,5> DelayUInt16_Size5;
typedef Delay<uint16_t,6> DelayUInt16_Size6;
typedef Delay<uint16_t,7> DelayUInt16_Size7;

typedef Delay<int32_t,2> DelayInt32_Size2;
typedef Delay<int32_t,3> DelayInt32_Size3;
typedef Delay<int32_t,4> DelayInt32_Size4;
typedef Delay<int32_t,5> DelayInt32_Size5;
typedef Delay<int32_t,6> DelayInt32_Size6;
typedef Delay<int32_t,7> DelayInt32_Size7;
typedef Delay<uint32_t,2> DelayUInt32_Size2;
typedef Delay<uint32_t,3> DelayUInt32_Size3;
typedef Delay<uint32_t,4> DelayUInt32_Size4;
typedef Delay<uint32_t,5> DelayUInt32_Size5;
typedef Delay<uint32_t,6> DelayUInt32_Size6;
typedef Delay<uint32_t,7> DelayUInt32_Size7;

#endif



