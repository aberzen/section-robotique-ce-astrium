// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Copyright 2012 Andrew Tridgell, all rights reserved.

//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

#ifndef QUATERNION_HPP_
#define QUATERNION_HPP_

#include <math/Matrix3.hpp>

namespace math {

class Quaternion
{
public:

    // constructor creates a quaternion equivalent
    // to roll=0, pitch=0, yaw=0
    Quaternion();

    // Copy existing quaternion
    Quaternion(const Quaternion& q);

    // setting constructor
    Quaternion(const float& scal, const float& vectX, const float& vectY, const float& vectZ);

    // setting constructor
    Quaternion(const float& scal, const Vector3f& vect);

    // function call operator
    void operator ()(const float& scal, const float& vectX, const float& vectY, const float& vectZ);

    // function call operator
    void operator ()(const float& scal, const Vector3f& vect);

    // Copy existing quaternion
    void operator ()(const Quaternion& q);

    /** @brief Addition */
    static void add(const Quaternion& q1, const Quaternion& q2, Quaternion& qRes);
    Quaternion operator + (const Quaternion& q) const;

    /** @brief Substraction */
    static void substract(const Quaternion& q1, const Quaternion& q2, Quaternion& qRes);
    Quaternion operator - (const Quaternion& q) const;

    /** @brief Substract and store */
    Quaternion& operator -= (const Quaternion& q);

    /** @brief Addition and store */
    Quaternion& operator += (const Quaternion& q);

    /** @brief Multiplication */
    static void mult(const Quaternion& q1, const Quaternion& q2, Quaternion& qRes);
    Quaternion operator * (const Quaternion& q) const;

    /** @brief Conjugate */
    static void conj(Quaternion& q);
    Quaternion operator ~ () const;

    /** @brief Norm */
    float norm();

    /** @brief Normalize */
    float normalize(int8_t nIter = 3, float guess = 1.);

    /** @brief Multiply by scalar and store */
    Quaternion& operator *= (const float& scal);

    /** @brief Divide by scalar and store */
    Quaternion& operator /= (const float& scal);

    /** @brief Multiply by scalar */
    Quaternion operator * (const float& scal);

    /** @brief Divide by scalar */
    Quaternion operator / (const float& scal);

    /** @brief Multiply and store */
    Quaternion& operator *= (const Quaternion& q);

    /** @brief Rotation using
     * The rotation of V_I such that  V_B = rotate(V_I).
     *
     * The mathematical operation is:
     * V_B = (~q_BI) x V_I x q_BI
     *
     * Where q_BI is the rotation from B to I;
     */
    Vector3f rotateQconjVQ (const Vector3f& v_I) const;

    /** @brief Rotation using
     * The rotation of V_I such that  V_B = rotate(V_I).
     *
     * The mathematical operation is:
     * V_I = q_BI x V_B x (~q_BI)
     *
     * Where q_BI is the rotation from B to I;
     */
    Vector3f rotateQVQconj (const Vector3f& v_B) const;

    bool is_nan(void) const;
    void to_dcm(Matrix3f& dcm) const;
    void from_dcm(const Matrix3f& dcm);

    // create a quaternion from Euler angles
    void from_euler(const float& roll, const float& pitch, const float& yaw);

    // create eulers from a quaternion
    void to_euler(float& roll, float& pitch, float& yaw) const;


public:
    /** @brief Scalar part of the quaternion */
    float scalar;

    /** @brief Vector part of the quaternion */
    Vector3f vector;
};

}

#endif // QUATERNION_HPP_
