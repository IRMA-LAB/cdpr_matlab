clear all
close all
clc

f = 10;
zeta= 0.2;
i=0;
time = 0:0.001:10;
for t=time
   i=i+1;
   v(i) = 100*exp(-zeta*2*pi*f*t).*cos(2*pi*f*(sqrt(1-zeta^2))*t);
end
PlotFreqSimp(v')