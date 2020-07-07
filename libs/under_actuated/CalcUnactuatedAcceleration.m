function ucdpr_v = CalcUnactuatedAcceleration(ucdpr_v,ucdpr_p)

load = zeros(length(ucdpr_v.total_load_ss_a)+length(ucdpr_v.total_load_ss_u),1);
load(ucdpr_p.actuated_mask) = ucdpr_v.total_load_ss_a;
load(ucdpr_p.unactuated_mask) = ucdpr_v.total_load_ss_u;
A = ucdpr_v.analitic_orthogonal'*ucdpr_v.mass_matrix_global_ss_u;
b = ucdpr_v.analitic_orthogonal'*(load-ucdpr_v.mass_matrix_global_ss_a*ucdpr_v.actuated_deriv_2);
ucdpr_v.unactuated_deriv_2 = linsolve(A,b);
    
end