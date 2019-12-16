function mat = Poly5MatT(t)

mat = [1 t   t^2   t^3    t^4     t^5;
       0 1 2*t   3*t^2  4*t^3   5*t^4;
       0 0   2   6*t   12*t^2  20*t^3;
       0 0   0     6   24*t    60*t^2];
end