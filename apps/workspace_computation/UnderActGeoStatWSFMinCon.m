function val = UnderActGeoStatWSFMinCon(cdpr_p,cdpr_v,ut,pose,s)

pose(6,1) = s;
cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.ExtractVars(cdpr_p.underactuated_platform,0,pose);
cdpr_v.underactuated_platform.unactuated = fsolve(@(v) FunGsNoCheck(cdpr_p,cdpr_v.underactuated_platform.actuated,v),...
    cdpr_v.underactuated_platform.unactuated,ut.fsolve_options_grad);

pose_n = cdpr_v.underactuated_platform.RecomposeVars(0,cdpr_p.underactuated_platform);

cdpr_v = UpdateIKZeroOrd(pose_n(1:3),pose_n(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);

val = -min(cdpr_v.tension_vector);

end