function [c,ceq] = UnderActGeoStatWSFMinConConstr(cdpr_p,cdpr_v,ut,pose,tau_lim,s)

pose(6,1) = s;
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,pose);
cdpr_v.underactuated_platform.unactuated = fsolve(@(v) FunGsNoCheck(cdpr_p,cdpr_v.underactuated_platform.actuated,v),...
    cdpr_v.underactuated_platform.unactuated,ut.fsolve_options_grad);

pose_n = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);

cdpr_v = UpdateIKZeroOrd(pose_n(1:3),pose_n(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);

cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians...
    (cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
K_matrix = CalcStiffnessMatUnder(cdpr_v);
[~,ceq] = chol(K_matrix);
ceq = [ceq;FunGsNoCheck(cdpr_p,cdpr_v.underactuated_platform.actuated,cdpr_v.underactuated_platform.unactuated)];
c = -(cdpr_v.tension_vector-tau_lim(1));
c = [c; -(-cdpr_v.tension_vector+tau_lim(2))];

end