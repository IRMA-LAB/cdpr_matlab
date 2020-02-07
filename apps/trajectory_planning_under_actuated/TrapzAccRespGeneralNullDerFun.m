function v = TrapzAccRespGeneralNullDerFun(x,f)

v = [];
a = x(1);
T = x(2);
for i=1:length(f)
c = 2./((1-a)*a*(pi*f(i)*T)^2);
b = (1-a)*pi*f(i)*T;
g = a*pi*f(i)*T;
v=[v; c*sign(sin(b)*sin(g))*(-sin(b)*sin(g)+b*cos(b)*sin(g)+g*sin(b)*cos(g))];
end

end