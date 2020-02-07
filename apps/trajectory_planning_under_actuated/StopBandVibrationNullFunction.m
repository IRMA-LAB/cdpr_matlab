function [vect,jac] = StopBandVibrationNullFunction(x,f,z,n_impulse)

shaper = [x(1:(length(x)+1)/2)';0 x((length(x)+1)/2+1:end)'];
shaperL = length(shaper);
vect = sum(shaper(1,:))-1;
jac = [ones(1,shaperL) zeros(1,shaperL-1)];

for i=1:2
    for j=0:1
    [v,jm] = ZeroVibrationConstrantNthOrder(2*pi*f(i),z(i),shaper,shaperL,j);
    vect = [vect;v];
    jac = [jac;jm];
    end
end

l = n_impulse-5;

for i=1:l
    if (mod(i,2)==0)
        [v,jm] = ZeroVibrationConstrantNthOrder(2*pi*mean(f),z(i),shaper,shaperL,1);
    else
        [v,jm] = ZeroVibrationConstrantNthOrder(2*pi*mean(f),z(i),shaper,shaperL,0);
    end
    vect = [vect;v];
    jac = [jac;jm];
end

end