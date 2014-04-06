// Unit vector
// 
// Normalize a vector by dividing by its norm2 value.
//
// INTPUT
// - v: a vector
//
// OUTPUT
// - v_unit: the unit vector
//
// USAGE
// res = vect_scalProd(lambda, vect);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [res] = vect_scalProd(lambda, vect)
	nRow = size(vect,1);
	nCol = size(vect,2);
	res = zeros(nRow,nCol);

	for iRow = 1:nRow
		res(iRow,:) = lambda .* vect(iRow,:);
	end
endfunction
