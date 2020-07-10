function vv = ComputePlot(x,v,n)

num_var = length(x)/n;
l = length(v);
vv = zeros(length(v),1);
t = 0;
for i=1:l
    for j=1:n
       vv(i) = vv(i)+x(4*(j-1)+1).*exp(-2*pi*x(4*(j-1)+2)*x(4*(j-1)+3).*t).*cos(2*pi*x(4*(j-1)+3).*t+x(4*(j-1)+4)); 
    end
    t = t+0.01;
end

end
