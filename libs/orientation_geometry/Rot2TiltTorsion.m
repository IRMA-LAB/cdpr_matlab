function angle = Rot2TiltTorsion(R,aPrev)
%ROT2TiltTorsion computes the magnitude of the rotations that result the matrix R.

angle(2,1) = acos(R(3,3));

if (aPrev(1)<-pi)
  angle(1,1) = atan(R(2,3)/R(1,3))-2*pi;
elseif(aPrev(1)>pi)
  angle(1,1) = atan(R(2,3)/R(1,3))+2*pi;
else
  angle(1,1) = atan(R(2,3)/R(1,3)); 
end

if (angle(1,1)-aPrev(1)>pi/2)
  angle(1,1) = angle(1,1) -2*pi;

elseif (angle(1,1)-aPrev(1)<-pi/2)
  angle(1,1) = angle(1,1) +2*pi;
end

angle(3,1) = angle(1,1)+atan2(R(3,2)/sin(angle(2,1)),-R(3,1)/sin(angle(2,1)));

s_th = (R(1,3)+R(2,3))/(cos(angle(1))+sin(angle(1)));
if (s_th<0)
  angle(2,1) = -abs(angle(2,1));
  angle(3,1) = angle(3,1)+pi;
end

% if(abs(angle(2,1))<0.001)
%     angle(1,1) = aPrev(1);
%     angle(3,1) = angle(1,1)+atan2(-sin(angle(1))*R(1,1)+cos(angle(1))*R(2,1),-sin(angle(1))*R(1,2)+cos(angle(1))*R(2,2));
% else
%     %angle(1,1) = atan2(R(2,3)/sin(angle(2,1)),R(1,3)/sin(angle(2,1)));
%     
% %     if(angle(1)<-pi)
% %       angle(1) = angle(1)+pi;
% %       angle(2) = -angle(2);
% %     elseif (angle(1)>pi)
% %       angle(1) = angle(1)-pi;
% %       angle(2) = -angle(2);
% %     end
%     angle(3,1) = angle(1,1)+atan2(R(3,2)/sin(angle(2,1)),-R(3,1)/sin(angle(2,1)));
% end
   
% angle(2,1) = atan2(((R(1,3)+R(2,3))/(cos(angle(1))+sin(angle(1)))+...
%   (R(3,1)+R(3,2))/(-cos(angle(3)-angle(1))+sin(angle(3)-angle(1))))/2,R(3,3));

end