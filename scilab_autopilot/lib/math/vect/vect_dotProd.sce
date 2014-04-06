// Dot product
// 
// Computed as the sum of the product of each coordinate.
//
// INTPUT
// - v1: a vector
// - v2: another vector
//
// OUTPUT
// - res: the dot product
//
// USAGE
// res = vect_dotProd(v1,v2);
//
// HISTORY
// 28/03/2014: T. Pareaud - Creation

function [res] = vect_dotProd(v1, v2)
	nRow = size(v1,1);
	nCol = max(size(v1,2),size(v2,2));
	res = zeros(1,nCol);
	for iRow = 1:nRow
		res = res + v1(iRow,:).*v2(iRow,:);
	end
endfunction
