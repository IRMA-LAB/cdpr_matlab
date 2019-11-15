function cdpr_v = UpdateIKFirstOrd(velocity,orientation_d,cdpr_p,cdpr_v)
%UPDATEIKFIRSTORD updates time dependent variables connected to the 1th order inverse kinematics problem of the cdpr. 
%  
%   UPDATEIKFIRSTORD updates time dependent variables of the platform and  
%   each cable storing them in the structure CDPR_V.
%   VELOCITY is a vector(size[3,1],[m/s]), containing the components of the
%   1th order time derivative of the position vector (P-O), projected on  
%   the global frame.
%   ORIENTATION_D can be a vector(size[3,1],[rad/s]), containing the 1th 
%   order time derivatives of angles of rotation or a vector (size[4,1]),
%   containing the components of the 1th order time derivatives of the
%   quaternion used to parameterize the platform orientation.
%   CDPR_P is a structure containing static parameters of the cdpr.
%   CDPR_V is a structure containing time dependent variable of the cdpr.

cdpr_v.platform = UpdatePlatformVelocity(velocity,orientation_d,...
  cdpr_p.rotation_parametrization,cdpr_v.platform);
for i=1:length(cdpr_v.cable)
  cdpr_v.cable(i) = UpdateCableFirstOrd(cdpr_p.cable(i),cdpr_v.platform,cdpr_v.cable(i));
end
cdpr_v.UpdateJacobiansD();
end