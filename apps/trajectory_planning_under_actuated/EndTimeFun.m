function [v,grad] = EndTimeFun(x)

v = x(end);
%v = 0;
grad = zeros(length(x),1);
grad(end) = 1;