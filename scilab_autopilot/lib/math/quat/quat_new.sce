// Create a quaternion
//
// This function create a quaternion ensuring correct
// convention of scalar/vectorial part.
//
// INTPUT
// - scal: scalar part
// - vect: vectorial part
//
// OUTPUT
// - qRes: quaternion with the scalar and vectorial part
//
// USAGE
// qRes = quat_new(scal, vect);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [qRes] = quat_new(scal, vect)
    qRes = [scal ; vect];
endfunction
