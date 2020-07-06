function f = IndetifyF(x,v,n)

num_var = length(x)/n;
l = length(v);
t = 0;
f = [];
for i=1:l
    if (~isnan(v(i)))
        f = [f;-v(i)];
        for j=1:n
        f(end) = f(end)+x(4*(j-1)+1).*exp(-2*pi*x(4*(j-1)+2)*x(4*(j-1)+3).*t).*cos(2*pi*x(4*(j-1)+3).*t+x(4*(j-1)+4)); 
        end
    end
    t = t+0.01;
end

end
