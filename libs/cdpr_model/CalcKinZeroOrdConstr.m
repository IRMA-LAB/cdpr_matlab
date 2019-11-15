function [cdpr_v,constraint] = CalcKinZeroOrdConstr(position,orientation,l,cdpr_p,cdpr_v)
%CALCKINZEROORDCONSTR updates each variable connected to the 
%   0th order kinematics problem of the cable robot and evaluates 
%   kinematic constraints. 
%
%   CALCKINZEROORDCONSTR updates each time dependent variable connected to
%   0th order kinematic of the cable robot and store them 
%   in proper structures. In addition, it evaluates the kinematic
%   constraint of zero order
%
%   CABLE_P is a structure containing static parameters of the cable and 
%   its swivel pulley.
%   PLATFORM_v is a structure containing time dependent variables of the 
%   platform.
%   CABLE_V is a structure containing time dependent variables of the 
%   cable and its swivel pulley.
%   CONSTR is an array containing the value of the kinematic constraint

cdpr_v.platform = UpdatePlatformPose(position,orientation,...
  cdpr_p.rotation_parametrization,cdpr_p.platform.pos_G_loc,cdpr_v.platform);
constraint = zeros(cdpr_p.n_cables,1);
for i=1:length(cdpr_v.cable)
  cdpr_v.cable(i) = UpdateCableZeroOrd(cdpr_p.cable(i),cdpr_v.platform,cdpr_v.cable(i));
  constraint(i,1) = cdpr_v.cable(i).complete_length-l(i);
end
cdpr_v = cdpr_v.UpdateJacobians();

end