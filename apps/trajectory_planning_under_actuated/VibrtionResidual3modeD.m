function val = VibrtionResidual3modeD(Shaper,len,w,zeta)

C = 0;
S = 0;
C1 = 0;
S1 = 0;
C2 = 0;
S2 = 0;

for i=1:len
    C = C + Shaper(1,i)*exp(zeta*w*Shaper(2,i))*cos(w*sqrt(1-zeta^2)*Shaper(2,i));
    S = S + Shaper(1,i)*exp(zeta*w*Shaper(2,i))*sin(w*sqrt(1-zeta^2)*Shaper(2,i));
    C1 = C1 -Shaper(2,i)*Shaper(1,i)*exp(zeta*w*Shaper(2,i))*sin(w*sqrt(1-zeta^2)*Shaper(2,i));
    S1 = S1 -Shaper(2,i)*Shaper(1,i)*exp(zeta*w*Shaper(2,i))*cos(w*sqrt(1-zeta^2)*Shaper(2,i));
    C2 = C2 + -Shaper(2,i)*Shaper(2,i)*Shaper(1,i)*exp(zeta*w*Shaper(2,i))*cos(w*sqrt(1-zeta^2)*Shaper(2,i));
    S2 = S2 + -Shaper(2,i)*Shaper(2,i)*Shaper(1,i)*exp(zeta*w*Shaper(2,i))*sin(w*sqrt(1-zeta^2)*Shaper(2,i));
end

SS = sqrt(C^2+S^2);
val = 0;
val = val + C1;
% val = val - (C*C1+S*S1)^2/SS^3;
% val = val + (C*C2+S*S2+C1^2+S1^2)/SS;
    
end