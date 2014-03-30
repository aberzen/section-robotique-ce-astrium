// Get the scalar part of a quaternion
//
// This function ensures that the correct index is used
// to retrieve the scalar part of the quaternion.
//
// INTPUT
// - q: input quaternion
//
// OUTPUT
// - scal: scalar part of the input quaternion
//
// USAGE
// scal = quat_getScal(q);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [scal] = quat_getScal(q)
    scal=q(1,:);
endfunction
