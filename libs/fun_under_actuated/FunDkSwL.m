function [vector,matrix] = FunDkSwL(r,cdpr_p,complete_length,swivel_angles,variables)

cdpr_v = CdprVar(cdpr_p.n_cables);
sw_constraint = zeros(cdpr_p.n_cables,1);
[cdpr_v,l_constraint] = CalcKinZeroOrdConstr(variables(1:3),variables(4:end),complete_length,cdpr_p,cdpr_v);
for i=1:cdpr_p.n_cables
    sw_constraint(i) = cdpr_v.cable(i).swivel_angle-swivel_angles(i);
end

l_jacobian = CalcJacobianL();
sw_jacobian = CalcJacobianSw();
vector = [sw_constraint;l_constraint];
matrix = [sw_jacobian;l_jacobian];

r.SetFrame(cdpr_v,cdpr_p);

end