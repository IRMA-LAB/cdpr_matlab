function [vector,matrix] = FunDkGsSwL(cdpr_p,complete_length,swivel_angles,variables,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
l_constraint = zeros(cdpr_p.n_cables,1);
sw_constraint = l_constraint;

cdpr_v = UpdateIKZeroOrd(variables(1:3,1),variables(4:end,1),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v.underactuated_platform =...
cdpr_v.underactuated_platform.UpdateStatics(cdpr_p.underactuated_platform,...
cdpr_v.analitic_jacobian,cdpr_v.D_mat,cdpr_v.platform.ext_load);
[cdpr_v.tension_vector,gs_constraint] = cdpr_v.underactuated_platform.CalcStaticTension(cdpr_p.underactuated_platform);

    for i=1:cdpr_p.n_cables
        l_constraint(i) = cdpr_v.cable(i).complete_length-complete_length(i);
        sw_constraint(i) = cdpr_v.cable(i).swivel_ang-swivel_angles(i);
    end

    l_jacobian = CalcJacobianL(cdpr_v);
    sw_jacobian = CalcJacobianSw(cdpr_v);
    gs_jacobian = CalcJacobianGs(cdpr_v);
    vector = [l_constraint;sw_constraint;gs_constraint];
    matrix = [l_jacobian;sw_jacobian;gs_jacobian];
    
if (~isempty(varargin))
varargin{1}.SetFrame(cdpr_v,cdpr_p);
end

end