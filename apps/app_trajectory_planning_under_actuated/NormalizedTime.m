function v = NormalizedTime(k,t,T)

 x = t/T;
 
 h = k(1)+k(2)*t+k(3)*t^2+k(4)*t^3+k(5)*t^4+k(6)*t^5;    
 hp = k(2)+2*k(3)*t+3*k(4)*t^2+4*k(5)*t^3+5*k(6)*t^4;
 hpp = 2*k(3)+6*k(4)*t+12*k(5)*t^2+20*k(6)*t^3;
 
 v(1) = x*(1+(1-x)*h);
 v(2) = x*(1-x)*hp+(1+(1-2*x)*h)/T;
 v(3) = x*(1-x)*hpp+2*((1-2*x)*hp-h/T)/T;

end