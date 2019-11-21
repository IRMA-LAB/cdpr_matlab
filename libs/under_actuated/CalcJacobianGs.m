function matrix = CalcJacobianGs(cdpr_v)

n = length(cdpr_v.cable);  
K = zeros(6);

for i=1:n
    T = CalcMatrixT(cdpr_v.cable(i));
    a_tilde = Anti(cdpr_v.cable(i).pos_PA_glob);
    K = K + cdpr_v.tension_vector(i).*[-T T*a_tilde;-a_tilde*T (a_tilde*T-Anti(cdpr_v.cable(i).vers_t))*a_tilde];
end

K(4:6,4:6) = K(4:6,4:6) + Anti(cdpr_v.platform.ext_load(1:3))*Anti(cdpr_v.platform.pos_PG_glob);
matrix = cdpr_v.underactuated_platform.geometric_orthogonal'*K;

matrix(:,4:end) = matrix(:,4:end)*cdpr_v.platform.H_mat;

end