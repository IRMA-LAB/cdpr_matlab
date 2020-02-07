function matrix = CalcMassMatTotalUnder(cdpr_v)

J = [cdpr_v.underactuated_platform.geometric_parallel,cdpr_v.underactuated_platform.geometric_orthogonal];

matrix = J'*cdpr_v.platform.mass_matrix_global*J;

matrix = (matrix+matrix')./2;

end