function res = bcfun(ya,yb,k,sim_data,i)

res = ya(1:end/2) - sim_data.p(4:6,i); 
res = [res; ya(end/2+1:end)];
res = [res; yb(1:end/2)-sim_data.p(4:6,i+1)];
res = [res; yb(end/2+1:end)];

end