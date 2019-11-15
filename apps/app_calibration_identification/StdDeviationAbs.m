function [devs] = StdDeviationAbs(A,b)

l1 = size(A);
l2 = size(b);
sigma_squared = ((A*b)'*(A*b))/(l1(1)-l2(1));
%sigma_squared = (s.^2)./(l1(1)-l2(1));
coeff = sigma_squared;
mat = A'*A;
for i=1:l2(1)
  c = zeros(l2(1),1);
  c(i,1) = 1;
  covariance_mat(:,i) =  coeff.*linsolve(mat,c);
end
devs = sqrt(diag(covariance_mat));

end