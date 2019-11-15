function cdpr_v = UpdateIKZeroOrd(position,orientation,cdpr_p,cdpr_v)
%UPDATEIKZEROORD updates time dependent variables connected to the 0th order inverse kinematics problem of the cdpr. 
%  
%   UPDATEIKZEROORD updates time dependent variables of the platform and of 
%   each cable storing them in the structure CDPR_V.
%   POSITION is a vector(size[3,1],[m]), containing the components of the
%   position vector (P-O), projected on the global frame.
%   ORIENTATION can be a vector(size[3,1],[rad]), containing the angles of
%   rotation or a vector (size[4,1]), containing the components of the
%   quaternion, used to parameterize the platform orientation.
%   CDPR_P is a structure containing static parameters of the cdpr.
%   CDPR_V is a structure containing time dependent variable of the cdpr.

cdpr_v.platform = UpdatePlatformPose(position,orientation,...
  cdpr_p.rotation_parametrization,cdpr_p.platform.pos_G_loc,cdpr_v.platform);
for i=1:length(cdpr_v.cable)
  cdpr_v.cable(i) = UpdateCableZeroOrd(cdpr_p.cable(i),cdpr_v.platform,cdpr_v.cable(i));
end
cdpr_v = cdpr_v.UpdateJacobians();

end