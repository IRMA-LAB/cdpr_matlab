function v = TrapzAccRespOptFun(x,f)

v=[];
for i=1:length(f)
v = [v;1/TrapzAccRespGeneral(x(1),x(2),f(i))];
end

end