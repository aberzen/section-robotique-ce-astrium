// Multiplication of two quaternion
//
// Hamilton multiplication.
//
// INTPUT
// - q1: first quaternion
// - q2: second quaternion
//
// OUTPUT
// - qRes: product of the two quaternions
//
// USAGE
// qRes = quat_mult(q1, q2);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [qRes] = quat_mult(q1, q2)
    qRes = [ ...
        quat_getScal(q1).*quat_getScal(q2) - vect_dotProd(quat_getVect(q1),quat_getVect(q2)) ; ...
        vect_scalProd(quat_getScal(q1),quat_getVect(q2)) + vect_scalProd(quat_getScal(q2),quat_getVect(q1)) +  vect_crossProd(quat_getVect(q1),quat_getVect(q2)) ];
endfunction
