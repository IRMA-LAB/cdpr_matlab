function [vect,jac] = MultiModeVibrationNullFunction(x,f,z,order)

l = length(f);
shaper = [x(1:(length(x)+1)/2)';0 x((length(x)+1)/2+1:end)'];
shaper(2,:) = shaper(2,:)-min(shaper(2,:));
    [shaper(2,:),idx] = sort(shaper(2,:));
    shaper(1,:) = shaper(1,idx);
shaperL = length(shaper);
vect = sum(shaper(1,:))-1;
jac = [ones(1,shaperL) zeros(1,shaperL-1)];

for i=1:l
   for j=1:order
       [v,jm] = ZeroVibrationConstrantNthOrder(2*pi*f(i),z(i),shaper,shaperL,j-1);
       vect = [vect;v];
       jac = [jac;jm];
   end
end

end