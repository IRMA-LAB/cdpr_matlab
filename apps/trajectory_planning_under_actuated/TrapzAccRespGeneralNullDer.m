function v = TrapzAccRespGeneralNullDer(a,T,f)

c = 2./((1-a)*a*(pi*f*T)^2);
b = (1-a)*pi*f*T;
g = a*pi*f*T;
v = c*sign(sin(b)*sin(g))*(-sin(b)*sin(g)+b*cos(b)*sin(g)+g*sin(b)*cos(g));

end