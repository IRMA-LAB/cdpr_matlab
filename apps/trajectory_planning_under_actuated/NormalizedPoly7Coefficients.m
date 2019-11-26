function coef = NormalizedPoly7Coefficients(T,coef)
%NORMALIZEDPOLY7COEFFICIENTS computes the coefficients of a normalized polynomial function of degree 7.
%
%   NORMALIZEDPOLY7COEFFICIENTS computes the nonzero coefficients of a
%   normalized polynomial function of degree 7, so that in 0 and T the 
%   function and its 1th,2nd and 3th order derivatives are equal to 0.
%   Moreover it returns the coefficients of the polynomial function and 
%   its 1th and 2nd order derivatives.   
%
%   T is the instant time selected for polynomial boundary conditions.
%   COEF is a structure containing the coefficients of the polynomial 
%   function and its derivatives. 
%
%   C is a vector (size[4,1]), field of the structure COEF containing the 
%   nonzeros polynomial coefficients.  
%   CDERIVATIVE is a vector (size[4,1]), field of the structure COEF 
%   containing the nonzero 1th order derivative polynomial coefficients. 
%   CDERIVATIVE2 is a vector (size[4,1]), field of the structure COEF 
%   containing the nonzero 2nd order derivative polynomial coefficients.
   

m=zeros(4);
t=zeros(4);
md=zeros(4);
for i = 1:4
  for j = 1:4
    if i == 1
      m(i,j) = 1;
    else
      m(i,j) = m(i-1,j)*(j+5-i);
    end
    t(i,j) = T^(j-i+4);
  end
  md(i,:) = m(i,:).*t(i,:);
end

for i = 1:4
  
end

b = zeros(4,1);
b(1) = 1;
c = linsolve(md,b);

coef.c = m(1,:)'.*c;
coef.cDerivative = m(2,:)'.*c;
coef.cDerivative2 = m(3,:)'.*c;

 end