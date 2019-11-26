function [vector,matrix] = HomingOptimizationFunction(cdpr_p,record,dl,ds,v)

dim = length(dl);
cdpr_v = CdprVar(cdpr_p.n_cables);

equations4stage = cdpr_p.n_cables+6;
vector = zeros(dim*equations4stage,1);
matrix = zeros(dim*equations4stage,length(v));
l_constraint = zeros(cdpr_p.n_cables,1);
sw_constraint = l_constraint;
for j = 1:dim
    
    pose = v(2*cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:2*cdpr_p.n_cables+j*cdpr_p.pose_dim,1);
    cdpr_v = UpdateIKZeroOrd(pose(1:3,1),pose(4:end,1),cdpr_p,cdpr_v);
    cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
    cdpr_v = CalcCablesStaticTension(cdpr_v);
    
    cdpr_v.underactuated_platform = cdpr_v.underactuated_platform.UpdateGeometricJacobians(cdpr_p.underactuated_platform,cdpr_v.geometric_jacobian);
    
    for i=1:cdpr_p.n_cables
        l_constraint(i) = cdpr_v.cable(i).complete_length-(v(i)+dl(j,i));
        sw_constraint(i) = cdpr_v.cable(i).swivel_ang-(v(i+cdpr_p.n_cables)+ds(j,i));
    end
    gs_constraint = cdpr_v.underactuated_platform.geometric_orthogonal'*cdpr_v.platform.ext_load;
    
    l_jacobian = CalcJacobianL(cdpr_v);
    sw_jacobian = CalcJacobianSw(cdpr_v);
    gs_jacobian = CalcJacobianGs(cdpr_v);
    constr_vect = [l_constraint;sw_constraint;gs_constraint];
    constr_jac = [l_jacobian;sw_jacobian;gs_jacobian];
    
    vector((j-1)*equations4stage+1:j*equations4stage,1) = constr_vect;
    matrix((j-1)*equations4stage+1:(j-1)*equations4stage+2*cdpr_p.n_cables,1:2*cdpr_p.n_cables) = -eye(2*cdpr_p.n_cables);
    matrix((j-1)*equations4stage+1:j*equations4stage,2*cdpr_p.n_cables+cdpr_p.pose_dim*(j-1)+1:2*cdpr_p.n_cables+j*cdpr_p.pose_dim) = constr_jac;
    
    if (j==1)
        record.SetFrame(cdpr_v,cdpr_p);
    end
end

end