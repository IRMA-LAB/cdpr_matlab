function cdpr_v = UpdateIKSecondOrd(acceleration,orientation_d_2,cdpr_p,cdpr_v)
%UPDATEIKSECONDORD updates time dependent variables connected to the 2th order inverse kinematics problem of the cdpr. 
%  
%   UPDATEIKSECONDORD updates time dependent variables of the platform and  
%   each cable storing them in the structure CDPR_V.
%   ACCELERATION is a vector(size[3,1],[m/s]), containing the components of the
%   2th order time derivative of the position vector (P-O), projected on  
%   the global frame.
%   ORIENTATION_D_2 can be a vector(size[3,1],[rad/s]), containing the 2th 
%   order time derivatives of angles of rotation or a vector (size[4,1]),
%   containing the components of the 2th order time derivatives of the
%   quaternion used to parameterize the platform orientation.
%   CDPR_P is a structure containing static parameters of the cdpr.
%   CDPR_V is a structure containing time dependent variable of the cdpr.

cdpr_v.platform = UpdatePlatformAcceleration(acceleration,orientation_d_2,...
  cdpr_p.rotation_parametrization,cdpr_v.platform);
for i=1:length(cdpr_v.cable)
  cdpr_v.cable(i) = UpdateCableSecondOrd(cdpr_v.platform,cdpr_v.cable(i));
end

end