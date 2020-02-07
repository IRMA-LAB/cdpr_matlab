function [vectmin,vect] = MultiModeVibrationNullFunctionPositive(x,f,z,order)

l = length(f);
shaper = [x(1:(length(x)+1)/2)';0 x((length(x)+1)/2+1:end)'];
shaperL = length(shaper);
vect = sum(shaper(1,:))-1;
for i=1:l
   for j=0:order
       vect = [vect;ZeroVibrationConstrantNthOrder(2*pi*f(i),z(i),shaper,shaperL,j)];
   end
end
vectmin = -x(1:(length(x)+1)/2);
j=1;
for i=(length(x)+1)/2+1:length(x)
    j=j+1;
    vectmin = [vectmin;shaper(2,j-1)-shaper(2,j)];
end

end