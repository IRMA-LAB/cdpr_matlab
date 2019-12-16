function [val,dval,ddval] = trapezLaw(t)

a = 9/2;
if(t<=1/3)
    val = 0.5*a*t.*t;
    dval = a*t;
    ddval = a;
elseif(t<=2/3)
    val = 1/4*(6*t-1);
    dval = a/3;
    ddval = 0;
else
    val = -1/4*(3*t-1)*(3*t-5);
    dval = a*(1-t);
    ddval = -a;
end

end