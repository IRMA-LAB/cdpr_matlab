function matrix = CalcMassMatUnder(cdpr_v)

matrix = cdpr_v.underactuated_platform.geometric_orthogonal'*...
    cdpr_v.platform.mass_matrix_global*cdpr_v.underactuated_platform.geometric_orthogonal;

matrix = (matrix+matrix')./2;

end