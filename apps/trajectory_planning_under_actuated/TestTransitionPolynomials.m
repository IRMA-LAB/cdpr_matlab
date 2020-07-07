function c = TestTransitionPolynomials(degree)

c = zeros(2*degree+1,1);
c_d = zeros(2*degree+1,1);
c_d_2 = zeros(2*degree+1,1);
sum_c = 0;
sum_c_d = 0;
sum_c_d_2 = 0;
for i=degree+1:2*degree+1
  c(i,1) = ((-1)^(i-degree-1)*factorial(2*degree+1))/...
    (i*factorial(degree)*factorial(i-degree-1)*factorial(2*degree+1-i));
   c_d(i-1,1) = c(i,1)*i;
   c_d_2(i-2,1) = c_d(i-1,1)*(i-1);
  sum_c = sum_c+c(i,1);
   sum_c_d = sum_c_d+c_d(i-1,1);
   sum_c_d_2 = sum_c_d_2+c_d_2(i-2,1);
end

ind = 0;
tVect = zeros(2*degree+1,1);
for tt=0:0.01:1
  ind = ind+1;
  t(ind) = tt;
  for j=1:2*degree+1
    tVect(j) = tt^j;
  end
  f(ind) = dot(c,tVect);
  f_d(ind) = dot(c_d,tVect);
  f_d_2(ind) = dot(c_d_2,tVect);
end

hold on
grid on
plot(t,f/(max(abs(f))))
plot(t,f_d/(max(abs(f_d))))
plot(t,f_d_2/(max(abs(f_d_2))))
hold off
end