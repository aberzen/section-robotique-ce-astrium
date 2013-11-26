/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * quaternion.cpp
 * Copyright (C) Andrew Tridgell 2012
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../include/MathUtils.hpp"
#include "../include/Quaternion.hpp"

namespace math {

// constructor creates a quaternion equivalent
// to roll=0, pitch=0, yaw=0
Quaternion::Quaternion() :
	_scalar(1.),
	_vector(0.,0.,0.) 
	{
}

// Copy existing quaternion
Quaternion::Quaternion(const Quaternion& q):
	_scalar(q._scalar),
	_vector(q._vector)
{
}

// setting constructor
Quaternion::Quaternion(const float q1, const float q2, const float q3, const float q4) :
	_scalar(q1),
	_vector(q2,q3,q4) 
	{
}

// setting constructor
Quaternion::Quaternion(const float scal, const Vector3f& vect) :
	_scalar(scal),
	_vector(vect) 
	{
}

// function call operator
void Quaternion::operator ()(const float q1, const float q2, const float q3, const float q4)
{
	_scalar = q1;
	_vector(q2,q3,q4);
}

// function call operator
void Quaternion::operator ()(const float scal, const Vector3f& vect)
{
	_scalar = scal;
	_vector(vect);
}

// Copy existing quaternion
void Quaternion::operator ()(const Quaternion& q)
{
	_scalar = q._scalar;
	_vector = q._vector;
}

// function addition
void Quaternion::add(const Quaternion& q1, const Quaternion& q2, Quaternion& qRes) {
	qRes._scalar = q1._scalar + q2._scalar;
	Vector3f::add(q1._vector, q2._vector, qRes._vector);
}

// function addition
Quaternion Quaternion::operator + (const Quaternion& q)
{
	return Quaternion(this->_scalar + q._scalar, this->_vector + q._vector);
}


// return the rotation matrix equivalent for this quaternion
void Quaternion::to_dcm(Matrix3f& m)
{
    float q3q3 = _vector.y * _vector.y;
    float q3q4 = _vector.y * _vector.z;
    float q2q2 = _vector.x * _vector.x;
    float q2q3 = _vector.x * _vector.y;
    float q2q4 = _vector.x * _vector.z;
    float q1q2 = _scalar * _vector.x;
    float q1q3 = _scalar * _vector.y;
    float q1q4 = _scalar * _vector.z;
    float q4q4 = _vector.z * _vector.z;

    m.a.x = 1.-2.*(q3q3 + q4q4);
    m.a.y =    2.*(q2q3 - q1q4);
    m.a.z =    2.*(q2q4 + q1q3);
    m.b.x =    2.*(q2q3 + q1q4);
    m.b.y = 1.-2.*(q2q2 + q4q4);
    m.b.z =    2.*(q3q4 - q1q2);
    m.c.x =    2.*(q2q4 - q1q3);
    m.c.y =    2.*(q3q4 + q1q2);
    m.c.z = 1.-2.*(q2q2 + q3q3);
}

// return the rotation matrix equivalent for this quaternion
void Quaternion::from_dcm(const Matrix3f& m)
{
	float den;
	float tmp1 = 1.+m.a.x+m.b.y+m.c.z;
	float tmp2 = 1.+m.a.x-m.b.y-m.c.z;
	if (tmp1>tmp2)
	{
		_scalar = 0.5 * sqrt(tmp1);
		den = (4.*_scalar);
		_vector.x = (m.c.y-m.b.z) / den ;
		_vector.y = (m.a.z-m.c.x) / den ;
		_vector.z = (m.b.x-m.a.y) / den ;
	}
	else
	{
		_vector.x = 0.5 * sqrt(tmp2);
		den = (4.*_vector.x);
		_vector.y = (m.a.y+m.b.x) / den ;
		_vector.z = (m.a.z+m.c.x) / den ;
		_scalar = (m.c.y-m.b.z) / den ;
	}
}

//// convert a vector from earth to body frame
//void Quaternion::earth_to_body(Vector3f& v)
//{
//    Matrix3f m;
//    // we reverse z before and afterwards because of the differing
//    // quaternion conventions from APM conventions.
//    v.z = -v.z;
//    rotation_matrix(m);
//    v = m * v;
//    v.z = -v.z;
//}

// create a quaternion from Euler angles
void Quaternion::from_euler(float roll, float pitch, float yaw)
{
    float cr2 = cos(roll*0.5);
    float cp2 = cos(pitch*0.5);
    float cy2 = cos(yaw*0.5);
    float sr2 = sin(roll*0.5);
    float sp2 = sin(pitch*0.5);
    float sy2 = sin(yaw*0.5);

    _scalar = cr2*cp2*cy2 + sr2*sp2*sy2;
    _vector.x = sr2*cp2*cy2 - cr2*sp2*sy2;
    _vector.y = cr2*sp2*cy2 + sr2*cp2*sy2;
    _vector.z = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// create eulers from a quaternion 3-1-3 convention
void Quaternion::to_euler(float& roll, float& pitch, float& yaw)
{
//	roll = (atan2(2.*(_scalar*_vector.x + _vector.y*_vector.z),
//				   1. - 2.*(_vector.x*_vector.x + _vector.y*_vector.y)));
//	// we let safe_asin() handle the singularities near 90/-90 in pitch
//	pitch = safe_asin(2.*(_scalar*_vector.y - _vector.z*_vector.x));
//
//	yaw = atan2(2.*(_scalar*_vector.z + _vector.x*_vector.y),
//				 1. - 2.*(_vector.y*_vector.y + _vector.z*_vector.z));
	roll = (atan2((_scalar*_vector.y + _vector.x*_vector.z),
				   -(_vector.x*_vector.y - _scalar*_vector.z)));
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	pitch = acos(-_scalar*_scalar-_vector.x*_vector.x+_vector.y*_vector.y+_vector.z*_vector.z);

	yaw = atan2((_scalar*_vector.y - _vector.x*_vector.z),
				(_vector.x*_vector.y + _scalar*_vector.z));
}



/*
// function call operator
void Quaternion::operator ()(const float scal, const float vectX, const float vectY, const float vectZ);

// function call operator
void Quaternion::operator ()(const float scal, const Vector3f& vect);
*/

Quaternion Quaternion::operator - (const Quaternion& q)
{
	return Quaternion(_scalar - q._scalar, _vector - q._vector);
}


/** @brief Substract and store */
Quaternion& Quaternion::operator -= (const Quaternion& q)
{
	_scalar -= q._scalar;
	_vector -= q._vector;
	return *this;
}

/** @brief Addition and store */
Quaternion& Quaternion::operator += (const Quaternion& q)
{
	_scalar += q._scalar;
	_vector += q._vector;
	return *this;
}

/** @brief Multiplication */
void Quaternion::mult(const Quaternion& q1, const Quaternion& q2, Quaternion& qRes)
{
	qRes._scalar = q1._scalar*q2._scalar - (q1._vector*q2._vector);
	qRes._vector = (q1._vector*q2._scalar) + (q2._vector*q1._scalar) + (q1._vector%q2._vector);
}

Quaternion Quaternion::operator * (const Quaternion& q)
{
	return Quaternion(
			_scalar*q._scalar - (_vector*q._vector),
			(_vector*q._scalar) + (q._vector*_scalar) + (_vector%q._vector));
}


/** @brief Conjugate */
void Quaternion::conj(Quaternion& q)
{
	q._vector *= (-1.);
}
Quaternion Quaternion::operator ~ ()
{
	return Quaternion(_scalar, _vector * (-1.));
}

/** @brief Norm */
float Quaternion::norm()
{
	return sqrt(_scalar*_scalar+_vector*_vector);
}

void Quaternion::normalize(Quaternion& q)
{
	float norm = q.norm();
	q._scalar /= norm;
	q._vector /= norm;
}

/** @brief Normalize */
Quaternion Quaternion::normalize()
{
	float norm = this->norm();
	return Quaternion(_scalar/norm, _vector/norm);
}

/** @brief Multiply and store */
Quaternion& Quaternion::operator *= (const Quaternion& q)
{
	_scalar = _scalar*q._scalar - (_vector*q._vector);
	_vector = (_vector*q._scalar) + (q._vector*_scalar) + (_vector%q._vector);
	return *this;
}

/** @brief Rotation using
 * The rotation of V_I such that  V_B = rotate(V_I).
 *
 * The mathematical operation is:
 * V_B = (~q_BI) x V_I x q_BI
 *
 * Where q_BI is the rotation from B to I;
 */
Vector3f Quaternion::rotateQconjVQ (const Vector3f& v_I)
{
	Quaternion  qVect_I(0., v_I);
	Quaternion  qVect_B = (~(*this))*qVect_I*(*this);
	return qVect_B._vector;
}

/** @brief Rotation using
 * The rotation of V_I such that  V_B = rotate(V_I).
 *
 * The mathematical operation is:
 * V_I = q_BI x V_B x (~q_BI)
 *
 * Where q_BI is the rotation from B to I;
 */
Vector3f Quaternion::rotateQVQconj (const Vector3f& v_B)
{
	Quaternion  qVect_B(0., v_B);
	Quaternion  qVect_I = (*this)*qVect_B*(~(*this));
	return qVect_I._vector;
}


bool Quaternion::is_nan(void)
{
	return isnan(_scalar) || _vector.is_nan();
}

/*
void Quaternion::rotation_matrix(Matrix3f& m);

// create a quaternion from Euler angles
void Quaternion::from_euler(float roll, float pitch, float yaw);

// create eulers from a quaternion
void Quaternion::to_euler(float& roll, float& pitch, float& yaw);
*/

}
