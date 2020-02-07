function v = TrapzAccRespGeneral(a,T,f)

v = 2./((1-a)*a*f*(pi*T)^2).*abs(sin((1-a)*pi*f*T)*sin(a*pi*f*T));

end