function v = TrapzAccResp2(w,f0)

a = 1/3;

v = 2.*f0./((1-a)*a*(pi^2*w)).*abs(sin((1-a)*pi*w)*sin(a*pi*w));

end