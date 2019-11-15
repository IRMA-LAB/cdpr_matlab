function ucdpr_v = CalcUnactuatedAcceleration(ucdpr_v)

load = [ucdpr_v.total_load_ss_a;ucdpr_v.total_load_ss_u];
ucdpr_v.unactuated_deriv_2 = ...
    linsolve(ucdpr_v.analitic_orthogonal'*ucdpr_v.mass_matrix_global_ss_u,...
    ucdpr_v.analitic_orthogonal'*(load-ucdpr_v.mass_matrix_global_ss_a*ucdpr_v.actuated_deriv_2));

end