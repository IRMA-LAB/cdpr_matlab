function cdpr_v = CalcCablesDynamicTensionStateSpace(cdpr_v)
%CALCCABLESTENSIONDYNUNDERACT computes the cables tension
%   CALCCABLESTENSIONDYN
%
%   CDPR_V is a structure containing time dependent variables of the cdpr.
%
%   VECTOR is the vector (size[])
A = cdpr_v.analitic_jacobian*cdpr_v.analitic_jacobian';
b = cdpr_v.analitic_jacobian*(-cdpr_v.platform.mass_matrix_global_ss*...
    cdpr_v.platform.pose_d_2+cdpr_v.platform.total_load_ss);
vector = linsolve(A,b);

% for i=1:length(vector)
%     if (vector(i)<=0)
%         vector(i)=NaN;
%     end
% end
cdpr_v.tension_vector = vector;
end