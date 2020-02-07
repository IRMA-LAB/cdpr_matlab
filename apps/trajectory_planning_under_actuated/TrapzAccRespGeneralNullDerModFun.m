function v = TrapzAccRespGeneralNullDerModFun(x,f)

v = [];
b = x(1);
g = x(2);
for i=1:length(f)
v=[v;(-sin(b*f(i))*sin(g*f(i))+(b*cos(b*f(i))*sin(g*f(i))+g*sin(b*f(i))*cos(g*f(i))))*f(i)];
end

end