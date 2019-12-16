function matrix = CalcMassMatAnalyticUnder(cdpr_v)

matrix = cdpr_v.underactuated_platform.analitic_orthogonal'*...
    cdpr_v.platform.mass_matrix_global_ss*cdpr_v.underactuated_platform.analitic_orthogonal;

matrix = (matrix+matrix')./2;

end