function matrix = CalcStiffnessMat(cdpr_v,cdpr_p)

n = length(cdpr_v.cable); m = length(cdpr_v.analitic_jacobian);
app_mat = zeros(6,m);

J_sw = CalcJacobianSw(cdpr_v);
J_tan = CalcJacobianTan(cdpr_v);

for i=1:n
   dt =  sin(cdpr_v.cable(i).tan_ang).*cdpr_v.cable(i).vers_w*J_sw(i,:)+...
       cdpr_v.cable(i).vers_n*J_tan(i,:);
   app_mat(1:3,:) =  app_mat(1:3,:)+dt.*cdpr_v.tension_vector(i);
   app_mat(4:6,:) = app_mat(4:6,:)+(Anti(cdpr_v.cable(i).pos_PA_glob)*dt-Anti(cdpr_v.cable(i).vers_t)*...
       [zeros(3) -Anti(cdpr_v.cable(i).pos_PA_glob)*cdpr_v.platform.H_mat]).*cdpr_v.tension_vector(i);
end

matrix = -cdpr_v.underactuated_platform.geometric_orthogonal'*...
    (-app_mat+cdpr_p.platform.mass.*[zeros(3,6);zeros(3,3) ...
    Anti(cdpr_p.platform.gravity_acceleration)*Anti(cdpr_v.platform.pos_PG_glob)*cdpr_v.platform.H_mat])*...
    cdpr_v.underactuated_platform.geometric_orthogonal;

end