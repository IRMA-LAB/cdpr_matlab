function [mat,l] = ZeroVibrationInputShapingNImpulse(n,T)

mat = zeros(2,n);
M = 0;
K = exp(-2*pi/(n));
for i=1:n-1
    M = M+K^i;
end
for i=1:n
   mat(1,i) = (K^(i-1))/(1+M);
   mat(2,i) = (i-1)*T/n;
end
l = length(mat);

end