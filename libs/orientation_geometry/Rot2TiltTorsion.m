function angle = Rot2TiltTorsion(R,aPrev)
%ROT2TiltTorsion computes the magnitude of the rotations that result the matrix R.

s_th = sqrt(1-R(3,3)^2);
if(s_th<0.0001)
    angle(1,1) = aPrev(1);
else
    angle(1,1) = atan2(R(2,3)./s_th,R(1,3)./s_th);
end
% if(angle(1,1)<-pi)
%    angle(1,1) = angle(1,1)+2*pi; 
% elseif(angle(1,1)>pi)
%    angle(1,1) = angle(1,1)-2*pi; 
% end
    
angle(2,1) = atan2(abs(cos(angle(1,1))*R(1,3)+sin(angle(1,1))*R(2,3)),R(3,3));
angle(3,1) = angle(1,1)+atan2(-sin(angle(1))*R(1,1)+cos(angle(1))*R(2,1),-sin(angle(1))*R(1,2)+cos(angle(1))*R(2,2));

end