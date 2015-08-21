// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	AverageFilter.h
/// @brief	A class to provide the average of a number of samples

#ifndef AverageFilter_h
#define AverageFilter_h

#include "FilterWithBuffer.hpp"

namespace autom {
// 1st parameter <T> is the type of data being filtered.
// 2nd parameter <U> is a larger data type used during summation to prevent overflows
// 3rd parameter <FILTER_SIZE> is the number of elements in the filter
template <class T, class U, uint8_t FILTER_SIZE>
class AverageFilter : public FilterWithBuffer<T,FILTER_SIZE>
{
public:
    // constructor
    AverageFilter();

    // Destructor
    virtual ~AverageFilter();

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T apply(T sample);

    // reset - clear the filter
    virtual void reset();

protected:
    uint8_t        _num_samples; // the number of samples in the filter, maxes out at size of the filter
};

/* Implementation and declaration of classes with template must be in the same file */
#include "../source/AverageFilter.cpp_"

// Typedef for convenience (1st argument is the data type, 2nd is a larger datatype to handle overflows, 3rd is buffer size)
typedef AverageFilter<int8_t,int16_t,2> AverageFilterInt8_Size2;
typedef AverageFilter<int8_t,int16_t,3> AverageFilterInt8_Size3;
typedef AverageFilter<int8_t,int16_t,4> AverageFilterInt8_Size4;
typedef AverageFilter<int8_t,int16_t,5> AverageFilterInt8_Size5;
typedef AverageFilter<uint8_t,uint16_t,2> AverageFilterUInt8_Size2;
typedef AverageFilter<uint8_t,uint16_t,3> AverageFilterUInt8_Size3;
typedef AverageFilter<uint8_t,uint16_t,4> AverageFilterUInt8_Size4;
typedef AverageFilter<uint8_t,uint16_t,5> AverageFilterUInt8_Size5;

typedef AverageFilter<int16_t,int32_t,2> AverageFilterInt16_Size2;
typedef AverageFilter<int16_t,int32_t,3> AverageFilterInt16_Size3;
typedef AverageFilter<int16_t,int32_t,4> AverageFilterInt16_Size4;
typedef AverageFilter<int16_t,int32_t,5> AverageFilterInt16_Size5;
typedef AverageFilter<uint16_t,uint32_t,2> AverageFilterUInt16_Size2;
typedef AverageFilter<uint16_t,uint32_t,3> AverageFilterUInt16_Size3;
typedef AverageFilter<uint16_t,uint32_t,4> AverageFilterUInt16_Size4;
typedef AverageFilter<uint16_t,uint32_t,5> AverageFilterUInt16_Size5;

typedef AverageFilter<int32_t,float,2> AverageFilterInt32_Size2;
typedef AverageFilter<int32_t,float,3> AverageFilterInt32_Size3;
typedef AverageFilter<int32_t,float,4> AverageFilterInt32_Size4;
typedef AverageFilter<int32_t,float,5> AverageFilterInt32_Size5;
typedef AverageFilter<uint32_t,float,2> AverageFilterUInt32_Size2;
typedef AverageFilter<uint32_t,float,3> AverageFilterUInt32_Size3;
typedef AverageFilter<uint32_t,float,4> AverageFilterUInt32_Size4;
typedef AverageFilter<uint32_t,float,5> AverageFilterUInt32_Size5;

typedef AverageFilter<float,float,5> AverageFilterFloat_Size5;


} /* namespace autom */
#endif
