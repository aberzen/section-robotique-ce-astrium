/*
 * nrd.h
 *
 *  Created on: 17 janv. 2013
 *      Author: Aberzen
 */

#include "../include/nrd.h"

/* **************************************************** */
/* Physics */
math::Vector3f g_I(0., 0., -PHYSICS_GRAVITY);

/* **************************************************** */
/* Hardware configuration */
math::Matrix3f cnfImuRateMat_UB(0.,1.,0.,-1.,0.,0.,0.,0.,1.);
math::Vector3f cnfImuRateBias_B(0.,0.,0.);
math::Matrix3f cnfImuAccMat_UB(0.,1.,0.,-1.,0.,0.,0.,0.,1.);
math::Vector3f cnfImuAccBias_B(0.,0.,0.);
math::Matrix3f cnfMagMat_UB(0.,0.,-1,1.,0.,0.,0.,-1.,0.);
math::Vector3f cnfMagBias_B(0.,0.,0.);



/* **************************************************** */
/* Attitude controller: angular rate only */


/* **************************************************** */
/* Attitude controller: angular attitude with rate bias */
