function [matrix,matrix_an] = CalcMassMatUnder(cdpr_v)

matrix = cdpr_v.underactuated_platform.geometric_orthogonal'*...
    cdpr_v.platform.mass_matrix_global*cdpr_v.underactuated_platform.geometric_orthogonal;
matrix_an = cdpr_v.underactuated_platform.analitic_orthogonal'*...
    cdpr_v.platform.mass_matrix_global_ss*cdpr_v.underactuated_platform.analitic_orthogonal;

matrix = (matrix+matrix')./2;
matrix_an = (matrix_an+matrix_an')./2;

end