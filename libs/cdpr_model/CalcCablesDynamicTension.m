function cdpr_v = CalcCablesDynamicTension(cdpr_v)
%CALCCABLESTENSIONDYNUNDERACT computes the cables tension
%   CALCCABLESTENSIONDYN
%
%   CDPR_V is a structure containing time dependent variables of the cdpr.
%
%   VECTOR is the vector (size[])
A = cdpr_v.geometric_jacobian*cdpr_v.geometric_jacobian';
b = cdpr_v.geometric_jacobian*(-cdpr_v.platform.mass_matrix_global*...
    [cdpr_v.platform.acceleration;cdpr_v.platform.angular_acc]+cdpr_v.platform.total_load);
vector = linsolve(A,b);

% for i=1:length(vector)
%     if (vector(i)<=0)
%         vector(i)=NaN;
%     end
% end
cdpr_v.tension_vector = vector;
end