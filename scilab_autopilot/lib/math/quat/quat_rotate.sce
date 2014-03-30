// Rotation of a vector using a unitary quaternion
//
// Rotate vector assuming that the quaternion encodes a rotation
// and thus is unitary.
// The rotaton is:
// v_A = q_AB * v_B * q_BA
// with * coding the Hamilton product.
//
// INTPUT
// - q_AB: quaternion encoding rotation from A to B
// - vect_B: vector coordinates in B
//
// OUTPUT
// - vect_A: vector coordinates in A
//
// USAGE
// vect_A = quat_rotate(q_AB,vect_B);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [vect_A] = quat_rotate(q_AB,vect_B)
	// v_A = q_AB * v_B * q_BA
    v_B = quat_new(zeros(1,size(vect_B,2)), vect_B);
	v_A = quat_mult( ...
				quat_mult(q_AB, v_B), ...
				quat_conj(q_AB) ...
	);

    vect_A = quat_getVect( v_A );
endfunction
