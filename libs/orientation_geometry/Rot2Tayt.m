function angle = Rot2Tayt(R,aPrev)
%ROT2TAYT

angle(2,1) = atan2(R(1,3),(sqrt(R(1,1)^2+R(1,2)^2)+sqrt(R(2,3)^2+R(3,3)^2))/2);

if cos(angle(2,1)) >= 0.0001 || cos(angle(2,1)) <= -0.0001
    angle(1,1) = atan2(-R(2,3),R(3,3));
    angle(3,1) = atan2(-R(1,2),R(1,1));
else
    % Todo
end
end