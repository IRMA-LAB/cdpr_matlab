function [dummy1,vect,dummy2,jac] = MultiModeVibrationFMINCONFunctionConstr(x,f,z,order)

l = length(f);
shaper = [x(1:(length(x)+1)/2)';0 x((length(x)+1)/2+1:end)'];
shaperL = length(shaper);
vect = [];
jac = [];

for i=1:l
   for j=1:order
       [v,jm] = ZeroVibrationConstrantNthOrder(2*pi*f(i),z(i),shaper,shaperL,j-1);
       vect = [vect;v];
       jac = [jac;jm];
   end
end
jac = jac';
dummy1 = [];
dummy2 = [];
end