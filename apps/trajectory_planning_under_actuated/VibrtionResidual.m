function val = VibrtionResidual(Shaper,len,w,zeta)

C = 0;
S = 0;

for i=1:len
    C = C + Shaper(1,i)*exp(zeta*w*Shaper(2,i))*cos(w*sqrt(1-zeta^2)*Shaper(2,i));
    S = S + Shaper(1,i)*exp(zeta*w*Shaper(2,i))*sin(w*sqrt(1-zeta^2)*Shaper(2,i));
end
val = exp(-zeta*w*Shaper(2,end))*sqrt(C^2+S^2);
    
end