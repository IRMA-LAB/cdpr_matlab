function [vect,jac] = ZeroVibrationConstrantNthOrder(w,z,shaper,shaperL,order)

vect = zeros(2,1);
jac = zeros(2,shaperL*2-1);
for i=1:shaperL
   vect = vect + [shaper(1,i)*shaper(2,i)^(order)*exp(z*w*shaper(2,i))*cos(w*sqrt(1-z^2)*shaper(2,i));shaper(1,i)*shaper(2,i)^(order)*exp(z*w*shaper(2,i))*sin(w*sqrt(1-z^2)*shaper(2,i))];
   jac(:,i) = shaper(2,i)^(order).*exp(z*w*shaper(2,i)).*...
       [cos(w*sqrt(1-z^2)*shaper(2,i));sin(w*sqrt(1-z^2)*shaper(2,i))];
   if i~=1
       jac(:,shaperL+i-1) = shaper(1,i).*shaper(2,i)^(order).*exp(-z*w*shaper(2,i)).*...
           (order/shaper(2,i).*[cos(w*sqrt(1-z^2)*shaper(2,i));sin(w*sqrt(1-z^2)*shaper(2,i))]+...
           w.*[cos(w*sqrt(1-z^2)*shaper(2,i))-sin(w*sqrt(1-z^2)*shaper(2,i));sin(w*sqrt(1-z^2)*shaper(2,i))+cos(w*sqrt(1-z^2)*shaper(2,i))]);
   end
end

end