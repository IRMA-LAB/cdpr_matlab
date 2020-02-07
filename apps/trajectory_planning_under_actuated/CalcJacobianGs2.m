function matrix = CalcJacobianGs2(cdpr_v)

n = length(cdpr_v.cable);  
K = zeros(6);

for i=1:n
    T = CalcMatrixT(cdpr_v.cable(i));
    a_tilde = Anti(cdpr_v.cable(i).pos_PA_glob);
    K = K + cdpr_v.tension_vector(i).*[-T T*a_tilde;-a_tilde*T (a_tilde*T-Anti(cdpr_v.cable(i).vers_t))*a_tilde];
end

K(4:6,4:6) = K(4:6,4:6) + Anti(cdpr_v.platform.ext_load(1:3))*Anti(cdpr_v.platform.pos_PG_glob);
D = eye(6); D(4:end,4:end) = cdpr_v.platform.H_mat;
matrix = cdpr_v.underactuated_platform.analitic_orthogonal'*D'*K*D*cdpr_v.underactuated_platform.analitic_orthogonal;

end