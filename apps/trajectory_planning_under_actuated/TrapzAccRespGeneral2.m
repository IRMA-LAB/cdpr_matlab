function v = TrapzAccRespGeneral2(a,fn,f)

v = 2.*(fn).^2./((1-a)*a*f*(pi)^2).*abs(sin((1-a)*pi*f/fn)*sin(a*pi*f/fn));

end