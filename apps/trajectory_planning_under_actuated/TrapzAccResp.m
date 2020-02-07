function v = TrapzAccResp(w,fn,a)

v = 2.*fn./((1-a)*a*(pi*w)^2).*abs(sin((1-a)*pi*w)*sin(a*pi*w));

end