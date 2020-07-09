function [vector,matrix] = FunDkStat(cdpr_p,cable_tensions,variables,varargin)

cdpr_v = CdprVar(cdpr_p.n_cables,cdpr_p.pose_dim);
cdpr_v = UpdateIKZeroOrd(variables(1:3),variables(4:end),cdpr_p,cdpr_v);
cdpr_v = CalcExternalLoads(cdpr_v,cdpr_p);
cdpr_v.tension_vector = cable_tensions;
vector = -cdpr_v.geometric_jacobian*cable_tensions+cdpr_v.platform.ext_load;
matrix = CalcJacobianStatic(cdpr_v);

if (~isempty(varargin))
varargin{1}.SetFrame(cdpr_v,cdpr_p);
end

end