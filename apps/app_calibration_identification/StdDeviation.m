function [devs] = StdDeviation(A,par,f)

l1 = size(A);
l2 = size(par);
sigma_squared = ((f-A*par)'*(f-A*par))/(l1(1)-l2(1));
%sigma_squared = (s.^2)./(l1(1)-l2(1));
coeff = sigma_squared;
mat = A'*A;
for i=1:l2(1)
  c = zeros(l2(1),1);
  c(i,1) = 1;
  covariance_mat(:,i) =  coeff.*linsolve(mat,c);
end
devs = 100.*sqrt(diag(covariance_mat))./par;

end