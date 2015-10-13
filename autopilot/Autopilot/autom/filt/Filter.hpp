// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file	Filter.h
/// @brief	A base class from which various filters classes should inherit
///

#ifndef Filter_h
#define Filter_h

#include <inttypes.h>

namespace autom {

template <class T>
class Filter
{
public:
    // constructor
    Filter();

    // destructor
    virtual ~Filter();

    // apply - Add a new raw value to the filter, retrieve the filtered result
    virtual T        apply(T sample);

    // reset - clear the filter
    virtual void        reset(T value);
};

/* Implementation and declaration of classes with template must be in the same file */
#include "Filter.cpp_"

} /* namespace autom */
#endif



