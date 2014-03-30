// Build a quaternion from a rotation
//
// Construct the quaternion encoding the expected rotation.
// 
// INTPUT
// - ang: rotation angle [rad]
// - vect: rotation direction
//
// OUTPUT
// - q: quaternion encoding the rotation of ang around the vect direction.
//
// USAGE
// [q] = rot2quat(ang, vect);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [q] = rot2quat(ang, vect)
    q = [cos(ang/2) ; vect_scalProd(sin(ang/2),vect)];
endfunction
