function [cdpr_v,constraint] = UnderactuatedStaticConstraintNoCheck(cdpr_v)


constraint = cdpr_v.underactuated_platform.geometric_orthogonal'*cdpr_v.platform.ext_load;
cdpr_v = CalcCablesStaticTensionNoCheck(cdpr_v);

end