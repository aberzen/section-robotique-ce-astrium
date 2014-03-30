// Get the vector part of a quaternion
//
// This function ensures that the correct index is used
// to retrieve the vector part of the quaternion.
//
// INTPUT
// - q: input quaternion
//
// OUTPUT
// - scal: vector part of the input quaternion
//
// USAGE
// vect = quat_getVect(q);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [vect] = quat_getVect(q)
    vect=q(2:4,:);
endfunction
