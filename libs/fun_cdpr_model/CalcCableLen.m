function complete_length = CalcCableLen(sw_r,ang_t,pos_BA_glob)
%CALCCABLELEN computes the length of the cable.
%   %CALCCABLELEN computes the length of the cable from the constraint
%   equation that links the norm of the position vector (A-B) with the
%   cable length, the pulley radius and the wrapped angle of the cable
%   inside the pulley groove.
%
%   SW_R is the pulley radius([m]).
%   ANG_T is the supplementary angle ([rad]) of the wrapped angle.
%   POS_BA_GLOB is a vector(size[3,1],[m]), containing  the components of
%   the position vector (A-B), projected on the global frame.


  complete_length = sw_r*(pi-ang_t)+sqrt(dot(pos_BA_glob,pos_BA_glob));
  
end