function [cdpr_v,constraint] = CalcKinSecondOrdConstr(acceleration,orientation_d2,l_d2,cdpr_p,cdpr_v)
%CALCKINFIRSTORDCONSTR updates time dependent variables connected to the 1th  
%   order inverse kinematics problem of the cdpr and evaluates kisnematic constraints. . 
%  
%   UPDATEIKFIRSTORD updates time dependent variables of the platform and  
%   each cable storing them in the structure CDPR_V. In addition, it
%   evaluates the first order differential kinematic constraint of the system.
%   VELOCITY is a vector(size[3,1],[m/s]), containing the components of the
%   1th order time derivative of the position vector (P-O), projected on  
%   the global frame.
%   ORIENTATION_D can be a vector(size[3,1],[rad/s]), containing the 1th 
%   order time derivatives of angles of rotation or a vector (size[4,1]),
%   containing the components of the 1th order time derivatives of the
%   quaternion used to parameterize the platform orientation.
%   CDPR_P is a structure containing static parameters of the cdpr.
%   CDPR_V is a structure containing time dependent variable of the cdpr.
%   CONSTR is an array containing the value of the first order differential
%   kinematic constraint

cdpr_v.platform = UpdatePlatformAcceleration(acceleration,orientation_d2,...
  cdpr_p.rotation_parametrization,cdpr_v.platform);
constraint = zeros(cdpr_p.n_cables,1);

for i=1:length(cdpr_v.cable)
  cdpr_v.cable(i) = UpdateCableSecondOrd(cdpr_v.platform,cdpr_v.cable(i));
  constraint(i,1) = cdpr_v.cable(i).complete_acceleration - l_d2(i);
end

end