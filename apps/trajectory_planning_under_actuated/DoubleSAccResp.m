function v = DoubleSAccResp(x,f)

fn = x(1);
a = x(2);
b = x(3);

v = 2.*fn^2./(pi^3*f^2).*abs(sin((1-lims)*pi*f*x)*sin(lims*pi*f*x));

end