function [X] = dyn_new(pos, vel, q, rate)
	X = [ pos ; vel ; q ; rate ];
endfunction
