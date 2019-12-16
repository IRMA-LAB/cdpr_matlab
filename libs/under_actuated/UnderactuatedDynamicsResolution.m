function cdpr_v = UnderactuatedDynamicsResolution(cdpr_v,ucdpr_p)
%UNDERACTUATEDDYNAMICSACCELERATION computes a subset of the acceleration vector of the cdpr.
%
%   UNDERACTUATEDDYNAMICSACCELERATION computes the 2nd order time
%   derivatives of the unactuated coordinates of an underactuated system.
%   The following expression arises from the partition of the system of
%   equations of dynamic equilibrium.
%
%   CDPR_V is a structure containing time dependent variables of the cdpr.

cdpr_v.underactuated_platform = CalcUnactuatedAcceleration(cdpr_v.underactuated_platform);
cdpr_v.platform.pose_d_2 = cdpr_v.underactuated_platform.RecomposeVars(2,ucdpr_p);
cdpr_v = CalcCablesDynamicTensionStateSpace(cdpr_v);

% for i=1:length(cdpr_v.cable)
%     if (isnan(cdpr_v.tension_vector(i)))
%         cdpr_v.underactuated_platform.unactuated_deriv_2=NaN(length(cdpr_v.underactuated_platform.unactuated_deriv_2),1);
%         break
%     end
% end
cdpr_v.platform.pose_d_2 = cdpr_v.underactuated_platform.RecomposeVars(2,ucdpr_p);

end