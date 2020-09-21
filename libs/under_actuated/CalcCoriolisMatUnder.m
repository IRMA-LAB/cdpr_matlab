function matrix = CalcCoriolisMatUnder(cdpr_v)

matrix = cdpr_v.underactuated_platform.geometric_orthogonal'*...
    (cdpr_v.platform.mass_matrix_global*cdpr_v.underactuated_platform.geometric_orthogonal_d+...
    cdpr_v.platform.C_mat*cdpr_v.underactuated_platform.geometric_orthogonal);

end